package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.climber.ClimberConstants;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ClimberSubsystem extends SubsystemBase {
  private static ClimberSubsystem instance;

  public static ClimberSubsystem getInstance() {
    if (instance == null) {
      instance = new ClimberSubsystem();
    }
    return instance;
  }

  private final ClimberInputsAutoLogged climberInputs = new ClimberInputsAutoLogged();

  private final SparkFlex climberFlex;

  private final DCMotor motors;

  private final SmartMotorControllerConfig climberMotorConfig;

  private final SmartMotorController climberMotor;

  private final ElevatorConfig climberConfig;

  private final Elevator climber;

  private Distance setpoint = ClimberConstants.setpoints.DEFAULT_HEIGHT;

  public ClimberSubsystem() {
    this.climberFlex = new SparkFlex(ClimberConstants.ID_climberMotor, MotorType.kBrushless);

    this.motors = DCMotor.getNeoVortex(1);

    this.climberMotorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                0,
                0,
                0,
                DegreesPerSecond.of(ClimberConstants.VELOCITY_M_PER_SECOND),
                DegreesPerSecondPerSecond.of(ClimberConstants.ACCELERATION_M_PER_SECOND_SQRD))
            .withSimClosedLoopController(
                0,
                0,
                0,
                DegreesPerSecond.of(ClimberConstants.VELOCITY_M_PER_SECOND),
                DegreesPerSecondPerSecond.of(ClimberConstants.ACCELERATION_M_PER_SECOND_SQRD))
            .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withTelemetry("ClimberMotor", TelemetryVerbosity.HIGH)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 3, 4)))
            .withMotorInverted(false)
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withMechanismCircumference(ClimberConstants.MECHANISM_CIRCUNFERENCE)
            .withControlMode(ControlMode.CLOSED_LOOP);

    this.climberMotor = new SparkWrapper(climberFlex, motors, climberMotorConfig);

    this.climberConfig =
        new ElevatorConfig(climberMotor)
            .withHardLimits(ClimberConstants.limits.MIN_HEIGHT, ClimberConstants.limits.MAX_HEIGHT)
            .withStartingHeight(ClimberConstants.limits.MIN_HEIGHT)
            .withMass(Mass.ofBaseUnits(2, Pounds))
            .withTelemetry("ClimberMechanism", TelemetryVerbosity.HIGH);

    this.climber = new Elevator(climberConfig);
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Subsystems/Climber", climberInputs);
    climber.updateTelemetry();
    this.setpoint =
        climber.getMechanismSetpoint().isEmpty()
            ? Meters.of(0)
            : Meters.of(
                climber.getMechanismSetpoint().get().in(Rotations)
                    * ClimberConstants.MECHANISM_CIRCUNFERENCE.in(Meters));
  }

  @Override
  public void simulationPeriodic() {
    climber.simIterate();
  }

  public Command holdPID() {
    return climber.setHeight(setpoint);
  }

  public Command climb() {
    return climber.setHeight(ClimberConstants.setpoints.CLIMBED_HEIGHT).until(isClimbed());
  }

  public Command defaultPosition() {
    return climber.setHeight(ClimberConstants.setpoints.DEFAULT_HEIGHT).until(isClimbed());
  }

  public Command raise() {
    return this.setHeight(ClimberConstants.setpoints.RAISED_HEIGHT);
  }

  public Command setHeight(Distance height) {
    return climber.setHeight(height).until(isClimbed());
  }

  public Trigger isClimbed() {
    return new Trigger(() -> climber.getHeight().isNear(setpoint, ClimberConstants.DEADBAND_IS_AT));
  }

  public Command setDutyCycle(double dutycycle) {
    return climber.set(dutycycle);
  }

  public void updateInputs() {
    climberInputs.climberAppliedVolts = climber.getMotor().getVoltage();
    climberInputs.climberCurrent = climber.getMotor().getStatorCurrent();
    climberInputs.climberPosition = climber.getMotor().getMeasurementPosition();
    climberInputs.climberSetpoint = setpoint;
    climberInputs.climberVelocity = climber.getMotor().getMeasurementVelocity();
  }
}
