package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ClimberConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
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
    public Angle climberPosition = Degrees.of(0);
    public AngularVelocity climberVelocity = DegreesPerSecond.of(0);
    public Angle climberDesiredPosition = Degrees.of(0);
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
          .withFeedforward(
              new SimpleMotorFeedforward(
                  ClimberConstants.feedforward.kS,
                  ClimberConstants.feedforward.kV,
                  ClimberConstants.feedforward.kA))
          .withSimFeedforward(
              new SimpleMotorFeedforward(
                  ClimberConstants.feedforward.kS,
                  ClimberConstants.feedforward.kV,
                  ClimberConstants.feedforward.kA))
          .withTelemetry("Climber", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 3, 4)))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motor = new SparkWrapper(climberMotor, motors, smcConfig);

  private final MechanismPositionConfig robotToMechanism =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(1.5))
          .withMaxRobotLength(Meters.of(0.75))
          .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)));

  private final PivotConfig m_config =
      new PivotConfig(motor)
          .withHardLimit(Degrees.of(0), Degrees.of(770.4))
          .withMechanismPositionConfig(robotToMechanism)
          .withStartingPosition(Degrees.of(0));

  private final Pivot climber = new Pivot(m_config);

  private Angle angleSetpoint = Degrees.of(0);

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
    return climber.setAngle(angleSetpoint);
  }

  public Command climb() {
    return climber.setAngle(ClimberConstants.setpoints.CLIMBED_ANGLE).until(isClimbed());
  }

  public Command defaultPosition() {
    return climber.setAngle(ClimberConstants.setpoints.DEFAULT_ANGLE).until(isClimbed());
  }

  public Command raise() {
    return climber.setAngle(ClimberConstants.setpoints.RAISED_ANGLE);
  }

  public Command setAngle(Angle angle) {
    angleSetpoint = angle;
    return climber.setAngle(angle).until(isClimbed());
  }

  public Trigger isClimbed() {
    return new Trigger(() -> climber.getAngle().isNear(null, null));
  }

  public Command setDutyCycle(double dutycycle) {
    return climber.set(dutycycle);
  }

  public void updateInputs() {
    climberInputs.climberAppliedVolts = climber.getMotor().getVoltage();
    climberInputs.climberCurrent = climber.getMotor().getStatorCurrent();
    climberInputs.climberPosition = climber.getMotor().getMechanismPosition();
    climberInputs.climberDesiredPosition = angleSetpoint;
    climberInputs.climberVelocity = climber.getMotor().getMechanismVelocity();
  }
}
