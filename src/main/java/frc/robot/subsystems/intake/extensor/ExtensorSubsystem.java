package frc.robot.subsystems.intake.extensor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.intake.IntakeConstants;
import org.littletonrobotics.junction.Logger;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ExtensorSubsystem extends SubsystemBase {
  private final ExtensorInputsAutoLogged extensorInputs = new ExtensorInputsAutoLogged();

  private final SparkFlex extensorSparkFlex =
      new SparkFlex(IntakeConstants.ID_intakeExtenderMotor, MotorType.kBrushless);

  private final SmartMotorControllerConfig extensorMotorConfig =
      new SmartMotorControllerConfig(this)
          .withMechanismCircumference(IntakeConstants.extensor.MECHANISM_CIRCUNFERENCE)
          .withClosedLoopController(
              new ExponentialProfilePIDController(
                  IntakeConstants.extensor.PID.kP,
                  IntakeConstants.extensor.PID.kI,
                  IntakeConstants.extensor.PID.kD,
                  ExponentialProfilePIDController.createConstraints(
                      Volts.of(10), RotationsPerSecond.of(2), RotationsPerSecondPerSecond.of(2))))
          .withGearing(IntakeConstants.extensor.gearing)
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("ExtensorMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withFeedforward(new SimpleMotorFeedforward(0, 4.4, 0.05))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController extensorMotor =
      new SparkWrapper(extensorSparkFlex, IntakeConstants.extensor.motor, extensorMotorConfig);

  private final ElevatorConfig extensorConfig =
      new ElevatorConfig(extensorMotor)
          .withMass(IntakeConstants.extensor.mass)
          .withSoftLimits(
              IntakeConstants.limits.MIN_EXTENSION, IntakeConstants.limits.MAX_EXTENSION)
          .withHardLimits(
              IntakeConstants.limits.MIN_EXTENSION, IntakeConstants.limits.MAX_EXTENSION)
          .withTelemetry("ExtensorMech", TelemetryVerbosity.HIGH)
          .withStartingHeight(Meters.of(0));

  private final Elevator extensor = new Elevator(extensorConfig);

  private Distance setpoint = Meters.of(0);

  public ExtensorSubsystem() {}

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Subsystems/Intake", extensorInputs);
    extensor.updateTelemetry();
    setpoint =
        Meters.of(
            extensorMotor.getMechanismPositionSetpoint().isEmpty()
                ? 0
                : extensorMotor.getMechanismPositionSetpoint().get().in(Rotations)
                    * IntakeConstants.extensor.MECHANISM_CIRCUNFERENCE.in(Meters));
  }

  @Override
  public void simulationPeriodic() {
    extensor.simIterate();
  }

  public void updateInputs() {
    extensorInputs.extensorVoltage = extensorMotor.getVoltage();
    extensorInputs.extensorCurrent = extensorMotor.getStatorCurrent();
    extensorInputs.extensorPosition = extensorMotor.getMeasurementPosition();
    extensorInputs.extensorVelocity = extensorMotor.getMeasurementVelocity();
    extensorInputs.extensorSetpoint = setpoint;
  }

  public Command extend() {
    return this.setLength(IntakeConstants.extensor.setpoints.INTAKE_EXTENDED_POSITION)
        .onlyWhile(isAtCollision())
        .andThen(softStopExtensor())
        .repeatedly();
  }

  public Command retract() {
    return this.setLength(IntakeConstants.extensor.setpoints.INTAKE_HOMED_POSITION);
  }

  public Command shake() {
    return this.extend()
        .until(isAtTargetAngle())
        .andThen(Commands.waitSeconds(0.5))
        .andThen(this.retract().until(isAtTargetAngle()))
        .andThen(Commands.waitSeconds(0.5))
        .repeatedly();
  }

  public Command softStopExtensor() {
    return disableExtensor()
        .andThen(this.setDutyCycle(0).withTimeout(Seconds.of(2.5)))
        .finallyDo(() -> extensorMotor.startClosedLoopController());
  }

  private Command disableExtensor() {
    return this.runOnce(
        () -> {
          extensorMotor.setIdleMode(MotorMode.COAST);
          extensorMotor.stopClosedLoopController();
        });
  }

  public Command homeReset(Current threshhold) {
    Debouncer currentDebouncer = new Debouncer(0.4);
    Voltage runVolts = Volts.of(-2);
    AngularVelocity velocityThreshold = DegreesPerSecond.of(2);
    return Commands.startRun(
            extensorMotor::stopClosedLoopController, () -> extensorMotor.setVoltage(runVolts))
        .until(
            () ->
                currentDebouncer.calculate(
                    extensorMotor.getStatorCurrent().gte(threshhold)
                        && extensorMotor.getMechanismVelocity().abs(DegreesPerSecond)
                            <= velocityThreshold.in(DegreesPerSecond)))
        .finallyDo(
            () -> {
              extensorMotor.setEncoderPosition(IntakeConstants.limits.MIN_EXTENSION);
              extensorMotor.startClosedLoopController();
            });
  }

  public Command holdPID() {
    return this.setLength(setpoint);
  }

  private Command setLength(Distance length) {
    return extensor.setHeight(length);
  }

  public Command setDutyCycle(double dutyCycle) {
    return extensor.set(dutyCycle);
  }

  public Trigger isAtCollision() {
    Debouncer isAtPositionDebouncer = new Debouncer(3, DebounceType.kFalling);
    return new Trigger(() -> isAtPositionDebouncer.calculate(isAtTargetAngle().getAsBoolean()));
  }

  public Trigger isAtTargetAngle() {
    return new Trigger(
        () -> extensor.getHeight().isNear(setpoint, IntakeConstants.extensor.DEADBAND_IS_AT));
  }
}
