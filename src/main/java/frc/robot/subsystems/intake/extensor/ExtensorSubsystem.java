package frc.robot.subsystems.intake.extensor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
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
            .withClosedLoopRampRate(Milliseconds.of(10))
            .withClosedLoopController(
                0.5, 0, 0, ExtensorConstants.maxVel, ExtensorConstants.maxAccel)
            .withSimClosedLoopController(
                0.5, 0, 0, ExtensorConstants.maxVel, ExtensorConstants.maxAccel)
            .withFeedforward(new SimpleMotorFeedforward(0.17, 0.2, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(0.17, 0.2, 0))
            .withGearing(ExtensorConstants.gearing)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(60))
            .withMotorInverted(false)
            .withControlMode(ControlMode.CLOSED_LOOP);

    this.extensorMotor =
        new SparkWrapper(extensorSparkFlex, ExtensorConstants.motor, extensorMotorConfig);

    this.extensorConfig =
        new ElevatorConfig(extensorMotor)
            .withMass(ExtensorConstants.mass)
            .withDrumRadius(ExtensorConstants.drumRadius)
            .withSoftLimits(
                ExtensorConstants.limits.MIN_EXTENSION, ExtensorConstants.limits.MAX_EXTENSION)
            .withHardLimits(
                ExtensorConstants.limits.MIN_EXTENSION, ExtensorConstants.limits.MAX_EXTENSION)
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
    return this.extending(Amps.of(30));
  }

  public Command retract() {
    return this.retracting(Amps.of(30));
  }

  public Command disableExtensor() {
    return this.runOnce(
        () -> {
          this.extensorMotor.stopClosedLoopController();
        });
  }

  public Command springyMode() {
    return this.runOnce(() -> extensorMotor.setVoltage(Voltage.ofBaseUnits(0, Volts)));
  }

  public Command enableExtensor() {
    return this.runOnce(
        () -> {
          this.extensorMotor.startClosedLoopController();
        });
  }

  /**
   * Reset the encoder to the lowest position when the current threshhold is reached. Should be used
   * when the Extensor position is unreliable, like startup. Threshhold is only detected if exceeded
   * for 0.4 seconds, and the motor moves less than 2 degrees per second.
   *
   * @param threshhold The current threshhold held when the Extensor is at it's hard limit.
   * @return
   */
  public Command retracting(Current threshhold) {
    Debouncer currentDebouncer =
        new Debouncer(0.3); // Current threshold is only detected if exceeded for 0.4 seconds.
    Voltage runVolts = Volts.of(12); // Volts required to run the mechanism to the limit.
    Distance limitHit =
        ExtensorConstants.setpoints.INTAKE_HOMED_POSITION; // Limit which gets hit.
    AngularVelocity velocityThreshold =
        DegreesPerSecond.of(
            2); // The maximum amount of movement for the extensor to be considered "hitting the
    // hard limit".
    return Commands.startRun(
            extensorMotor::stopClosedLoopController, // Stop the closed loop controller
            () -> extensorMotor.setVoltage(runVolts)) // Set the voltage of the motor
        .until(
            () ->
                currentDebouncer.calculate(
                    extensorMotor.getStatorCurrent().gte(threshhold)
                        && extensorMotor.getMechanismVelocity().abs(DegreesPerSecond)
                            <= velocityThreshold.in(DegreesPerSecond)))
        .finallyDo(
            () -> {
              extensorMotor.setEncoderPosition(limitHit);
              extensorMotor.startClosedLoopController();
            });
  }

  public Command extending(Current threshhold) {
    Debouncer currentDebouncer =
        new Debouncer(0.3); // Current threshold is only detected if exceeded for 0.4 seconds.
    Voltage runVolts = Volts.of(-12); // Volts required to run the mechanism to the limit.
    Distance limitHit =
        ExtensorConstants.setpoints.INTAKE_EXTENDED_POSITION; // Limit which gets hit.
    AngularVelocity velocityThreshold =
        DegreesPerSecond.of(
            2); // The maximum amount of movement for the extensor to be considered "hitting the
    // hard limit".
    return Commands.startRun(
            extensorMotor::stopClosedLoopController, // Stop the closed loop controller
            () -> extensorMotor.setVoltage(runVolts)) // Set the voltage of the motor
        .until(
            () ->
                currentDebouncer.calculate(
                    extensorMotor.getStatorCurrent().gte(threshhold)
                        && extensorMotor.getMechanismVelocity().abs(DegreesPerSecond)
                            <= velocityThreshold.in(DegreesPerSecond)))
        .finallyDo(
            () -> {
              extensorMotor.setEncoderPosition(limitHit);
              extensorMotor.startClosedLoopController();
            });
  }

  private Command setLength(Distance length) {
    return this.extensor.runTo(length, ExtensorConstants.DEADBAND_IS_AT);
  }

  public Trigger isAtTargetHeight() {
    return new Trigger(
        () -> extensor.getHeight().isNear(setpoint, ExtensorConstants.DEADBAND_IS_AT));
  }

  public Trigger isAtTargetHeightCollect() {
    return new Trigger(
        () ->
            extensor
                .getHeight()
                .isNear(
                    ExtensorConstants.setpoints.INTAKE_EXTENDED_POSITION,
                    ExtensorConstants.DEADBAND_IS_AT));
  }
}
