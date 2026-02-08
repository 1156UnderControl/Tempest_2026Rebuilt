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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.intake.ExtensorConstants;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
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

  private static ExtensorSubsystem instance;

  public static ExtensorSubsystem getInstance() {
    if (instance == null) {
      instance = new ExtensorSubsystem();
    }
    return instance;
  }

  private final ExtensorInputsAutoLogged extensorInputs;

  private final SparkFlex extensorSparkFlex;

  private final SmartMotorControllerConfig extensorMotorConfig;

  private final SmartMotorController extensorMotor;

  private final ElevatorConfig extensorConfig;

  private final Elevator extensor;

  private Distance setpoint = Meters.of(0);

  private ExtensorSubsystem() {
    this.extensorInputs = new ExtensorInputsAutoLogged();
    this.extensorSparkFlex = new SparkFlex(ExtensorConstants.ID_MOTOR, MotorType.kBrushless);

    this.extensorMotorConfig =
        new SmartMotorControllerConfig(this)
            .withMechanismCircumference(ExtensorConstants.MECHANISM_CIRCUNFERENCE)
            .withClosedLoopController(
                new ExponentialProfilePIDController(
                    4.5,
                    0,
                    0,
                    ExponentialProfilePIDController.createConstraints(
                        Volts.of(10), RotationsPerSecond.of(2), RotationsPerSecondPerSecond.of(2))))
            .withSimClosedLoopController(
                new ExponentialProfilePIDController(
                    4.5,
                    0,
                    0,
                    ExponentialProfilePIDController.createConstraints(
                        Volts.of(10), RotationsPerSecond.of(2), RotationsPerSecondPerSecond.of(2))))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("ExtensorMotor", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(false)
            .withFeedforward(new SimpleMotorFeedforward(0, 4.4, 0.05))
            .withSimFeedforward(new SimpleMotorFeedforward(0, 4.4, 0.05))
            .withControlMode(ControlMode.CLOSED_LOOP);

    this.extensorMotor =
        new SparkWrapper(extensorSparkFlex, DCMotor.getNeoVortex(1), extensorMotorConfig);

    this.extensorConfig =
        new ElevatorConfig(extensorMotor)
            .withMass(ExtensorConstants.mass)
            .withSoftLimits(
                ExtensorConstants.limits.MIN_EXTENSION, ExtensorConstants.limits.MAX_EXTENSION)
            .withHardLimits(
                ExtensorConstants.limits.MIN_EXTENSION, ExtensorConstants.limits.MAX_EXTENSION)
            .withTelemetry("ExtensorMech", TelemetryVerbosity.HIGH)
            .withStartingHeight(Meters.of(0));

    this.extensor = new Elevator(extensorConfig);
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Subsystems/Extensor", extensorInputs);
    extensor.updateTelemetry();
    setpoint =
        Meters.of(
            this.extensorMotor.getMechanismPositionSetpoint().isEmpty()
                ? 0
                : extensorMotor.getMechanismPositionSetpoint().get().in(Rotations)
                    * ExtensorConstants.MECHANISM_CIRCUNFERENCE.in(Meters));
  }

  @Override
  public void simulationPeriodic() {
    this.extensor.simIterate();
  }

  public void updateInputs() {
    extensorInputs.extensorVoltage = this.extensorMotor.getVoltage();
    extensorInputs.extensorCurrent = this.extensorMotor.getStatorCurrent();
    extensorInputs.extensorPosition = this.extensorMotor.getMeasurementPosition();
    extensorInputs.extensorVelocity = this.extensorMotor.getMeasurementVelocity();
    extensorInputs.extensorSetpoint = this.setpoint;
  }

  public Command extend() {
    return this.setLength(ExtensorConstants.setpoints.INTAKE_EXTENDED_POSITION)
        .onlyWhile(isAtCollision())
        .andThen(softStopExtensor())
        .repeatedly();
  }

  public Command retract() {
    return this.setLength(ExtensorConstants.setpoints.INTAKE_HOMED_POSITION);
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
        .andThen(this.setDutyCycle(0).withTimeout(Seconds.of(1.0)))
        .finallyDo(() -> this.extensorMotor.startClosedLoopController());
  }

  private Command disableExtensor() {
    return this.runOnce(
        () -> {
          this.extensorMotor.setIdleMode(MotorMode.COAST);
          this.extensorMotor.stopClosedLoopController();
        });
  }

  public Command homeReset(Current threshhold) {
    Debouncer currentDebouncer = new Debouncer(0.4);
    Voltage runVolts = Volts.of(-2);
    AngularVelocity velocityThreshold = DegreesPerSecond.of(2);
    return Commands.startRun(
            this.extensorMotor::stopClosedLoopController,
            () -> this.extensorMotor.setVoltage(runVolts))
        .until(
            () ->
                currentDebouncer.calculate(
                    this.extensorMotor.getStatorCurrent().gte(threshhold)
                        && this.extensorMotor.getMechanismVelocity().abs(DegreesPerSecond)
                            <= velocityThreshold.in(DegreesPerSecond)))
        .finallyDo(
            () -> {
              extensorMotor.setEncoderPosition(ExtensorConstants.limits.MIN_EXTENSION);
              extensorMotor.startClosedLoopController();
            });
  }

  public Command holdPID() {
    return this.setLength(setpoint);
  }

  public Command setLength(Distance length) {
    return this.extensor.setHeight(length);
  }

  public Command setDutyCycle(double dutyCycle) {
    return this.extensor.set(dutyCycle);
  }

  public Trigger isAtCollision() {
    Debouncer isAtPositionDebouncer = new Debouncer(3, DebounceType.kFalling);
    return new Trigger(() -> isAtPositionDebouncer.calculate(isAtTargetAngle().getAsBoolean()));
  }

  public Trigger isAtTargetAngle() {
    return new Trigger(
        () -> extensor.getHeight().isNear(setpoint, ExtensorConstants.DEADBAND_IS_AT));
  }
}
