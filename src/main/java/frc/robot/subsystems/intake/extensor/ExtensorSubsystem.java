package frc.robot.subsystems.intake.extensor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
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

  private final SparkFlex extensorMotor;

  private final SmartMotorControllerConfig smartExtensorMotorConfig;

  private final SmartMotorController smartExtensorMotor;

  private final ElevatorConfig extensorMechanismConfig;

  private final Elevator extensorMechanism;

  private Distance setpoint = Meters.of(0);

  public ExtensorSubsystem() {
    this.extensorMotor = new SparkFlex(IntakeConstants.ID_intakeExtenderMotor, MotorType.kBrushless);

    this.smartExtensorMotorConfig =
      new SmartMotorControllerConfig(this)
          .withMechanismCircumference(IntakeConstants.extensor.MECHANISM_CIRCUNFERENCE)
          .withClosedLoopController(
              new ExponentialProfilePIDController(
                  IntakeConstants.roller.PID.kP,
                  IntakeConstants.roller.PID.kI,
                  IntakeConstants.roller.PID.kD,
                  ExponentialProfilePIDController.createElevatorConstraints(
                      IntakeConstants.extensor.maxVolts,
                      IntakeConstants.extensor.motor,
                      IntakeConstants.extensor.mass,
                      IntakeConstants.extensor.drumRadius,
                      IntakeConstants.extensor.gearing)))
          .withGearing(IntakeConstants.extensor.gearing)
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("ExtensorMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withFeedforward(new SimpleMotorFeedforward(0, 4.4, 0.05))
          .withControlMode(ControlMode.CLOSED_LOOP);
    
    this.smartExtensorMotor =
      new SparkWrapper(this.extensorMotor, IntakeConstants.extensor.motor, this.smartExtensorMotorConfig);

    this.extensorMechanismConfig =
      new ElevatorConfig(smartExtensorMotor)
          .withMass(IntakeConstants.extensor.mass)
          .withSoftLimits(
              IntakeConstants.limits.MIN_EXTENSION, IntakeConstants.limits.MAX_EXTENSION)
          .withHardLimits(
              IntakeConstants.limits.MIN_EXTENSION, IntakeConstants.limits.MAX_EXTENSION)
          .withTelemetry("ExtensorMech", TelemetryVerbosity.HIGH)
          .withMechanismPositionConfig(IntakeConstants.extensor.robotToMechanism)
          .withStartingHeight(Meters.of(0));

    this.extensorMechanism = new Elevator(extensorMechanismConfig);
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Subsystems/Intake", this.extensorInputs);
    this.extensorMechanism.updateTelemetry();
    setpoint =
        Meters.of(
            this.smartExtensorMotor.getMechanismPositionSetpoint().isEmpty()
                ? 0
                : this.smartExtensorMotor.getMechanismPositionSetpoint().get().in(Rotations)
                    * IntakeConstants.extensor.MECHANISM_CIRCUNFERENCE.in(Meters));
  }

  @Override
  public void simulationPeriodic() {
    this.smartExtensorMotor.simIterate();
  }

  public void updateInputs() {
    extensorInputs.extensorVoltage = this.smartExtensorMotor.getVoltage();
    extensorInputs.extensorCurrent = this.smartExtensorMotor.getStatorCurrent();
    extensorInputs.extensorPosition = this.smartExtensorMotor.getMeasurementPosition();
    extensorInputs.extensorVelocity = this.smartExtensorMotor.getMeasurementVelocity();
    extensorInputs.extensorSetpoint = this.setpoint;
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
        .andThen(this.retract().until(isAtTargetAngle()))
        .repeatedly();
  }

  public Command softStopExtensor() {
    return disableExtensor()
        .andThen(this.setDutyCycle(0).withTimeout(Seconds.of(2.5)))
        .finallyDo(() -> this.smartExtensorMotor.startClosedLoopController());
  }

  private Command disableExtensor() {
    return this.runOnce(
        () -> {
          this.smartExtensorMotor.setIdleMode(MotorMode.COAST);
          this.smartExtensorMotor.stopClosedLoopController();
        });
  }

  public Command homeReset(Current threshhold) {
    Debouncer currentDebouncer = new Debouncer(0.4);
    Voltage runVolts = Volts.of(-2);
    AngularVelocity velocityThreshold = DegreesPerSecond.of(2);
    return Commands.startRun(
            this.smartExtensorMotor::stopClosedLoopController, () -> this.smartExtensorMotor.setVoltage(runVolts))
        .until(
            () ->
                currentDebouncer.calculate(
                    this.smartExtensorMotor.getStatorCurrent().gte(threshhold)
                        && this.smartExtensorMotor.getMechanismVelocity().abs(DegreesPerSecond)
                            <= velocityThreshold.in(DegreesPerSecond)))
        .finallyDo(
            () -> {
              this.smartExtensorMotor.setEncoderPosition(IntakeConstants.limits.MIN_EXTENSION);
              this.smartExtensorMotor.startClosedLoopController();
            });
  }

  public Command holdPID() {
    return this.setLength(setpoint);
  }

  private Command setLength(Distance length) {
    return this.extensorMechanism.setHeight(length);
  }

  public Command setDutyCycle(double dutyCycle) {
    return this.extensorMechanism.set(dutyCycle);
  }

  public Trigger isAtCollision() {
    Debouncer isAtPositionDebouncer = new Debouncer(3, DebounceType.kFalling);
    return new Trigger(() -> isAtPositionDebouncer.calculate(isAtTargetAngle().getAsBoolean()));
  }

  public Trigger isAtTargetAngle() {
    return new Trigger(
        () ->
            this.extensorMechanism
                .getHeight()
                .isNear(this.extensorMechanism.getHeight(), IntakeConstants.extensor.DEADBAND_IS_AT));
  }
}
