package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.climber.ClimberConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
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

  @AutoLog
  public static class ClimberInputs {
    public Distance climberPosition = Meters.of(0);
    public LinearVelocity climberVelocity = MetersPerSecond.of(0);
    public Distance climberDesiredPosition = Meters.of(0);
    public Voltage climberAppliedVolts = Volts.of(0);
    public Current climberCurrent = Amps.of(0);
  }

  private final ClimberInputsAutoLogged climberInputs = new ClimberInputsAutoLogged();

  private final SparkFlex climberMotor =
      new SparkFlex(ClimberConstants.ID_climberMotor, MotorType.kBrushless);

  private final DCMotor motors = DCMotor.getNeoVortex(1);

  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              ClimberConstants.PID.kP,
              ClimberConstants.PID.kI,
              ClimberConstants.PID.kD,
              DegreesPerSecond.of(ClimberConstants.VELOCITY_M_PER_SECOND),
              DegreesPerSecondPerSecond.of(ClimberConstants.ACCELERATION_M_PER_SECOND_SQRD))
          .withSimClosedLoopController(
              ClimberConstants.PID.kP,
              ClimberConstants.PID.kI,
              ClimberConstants.PID.kD,
              DegreesPerSecond.of(ClimberConstants.VELOCITY_M_PER_SECOND),
              DegreesPerSecondPerSecond.of(ClimberConstants.ACCELERATION_M_PER_SECOND_SQRD))
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withTelemetry("Climber", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 3, 4)))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withMechanismCircumference(Meters.of(ClimberConstants.MECHANISM_CIRCUNFERENCE))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motor = new SparkWrapper(climberMotor, motors, smcConfig);

  private final MechanismPositionConfig robotToMechanism =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(1.5))
          .withMaxRobotLength(Meters.of(0.75))
          .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)));

  private final ElevatorConfig m_config =
      new ElevatorConfig(motor)
          .withHardLimits(ClimberConstants.limits.MIN_HEIGHT, ClimberConstants.limits.MAX_HEIGHT)
          .withMechanismPositionConfig(robotToMechanism)
          .withStartingHeight(ClimberConstants.limits.MIN_HEIGHT)
          .withMass(Mass.ofBaseUnits(2, Kilograms));

  private final Elevator climber = new Elevator(m_config);

  private Distance heightSetpoint = ClimberConstants.setpoints.DEFAULT_HEIGHT;

  public ClimberSubsystem() {}

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Motors/Climber", climberInputs);
    climber.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    climber.simIterate();
  }

  public Command holdPID() {
    return climber.setHeight(heightSetpoint);
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
    heightSetpoint = height;
    return climber.setHeight(height).until(isClimbed());
  }

  public Trigger isClimbed() {
    return new Trigger(() -> climber.getHeight().isNear(heightSetpoint, ClimberConstants.DEADBAND));
  }

  public Command setDutyCycle(double dutycycle) {
    return climber.set(dutycycle);
  }

  public void updateInputs() {
    climberInputs.climberAppliedVolts = climber.getMotor().getVoltage();
    climberInputs.climberCurrent = climber.getMotor().getStatorCurrent();
    climberInputs.climberPosition = climber.getMotor().getMeasurementPosition();
    climberInputs.climberDesiredPosition = heightSetpoint;
    climberInputs.climberVelocity = climber.getMotor().getMeasurementVelocity();
  }
}
