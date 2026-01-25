package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.shooter.HoodConstants;
import frc.robot.constants.shooter.TurretConstants;
import frc.robot.subsystems.shooter.ShooterCalculator;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class TurretSubsystem extends SubsystemBase {

  private final SmartMotorControllerConfig motorConfig;
  private final SparkFlex motor;
  private final SmartMotorController smartMotor;
  private final PivotConfig mechanismConfig;
  private final Pivot mechanism;

  private final TurretInputsAutoLogged inputs;

  private static TurretSubsystem instance;

  private TurretSubsystem() {
    this.motorConfig =
        new SmartMotorControllerConfig(this)
            .withIdleMode(TurretConstants.MOTOR_MODE)
            .withGearing(TurretConstants.MOTOR_GEARING)
            .withMotorInverted(TurretConstants.Position.MOTOR_INVERTED)
            .withTelemetry(TurretConstants.MOTOR_NAME, TurretConstants.TELEMETRY)
            .withSoftLimit(
                TurretConstants.Position.TURRET_MIN_ANGLE,
                TurretConstants.Position.TURRET_MAX_ANGLE)
            .withStatorCurrentLimit(TurretConstants.STATOR_CURRENT)
            .withClosedLoopRampRate(TurretConstants.CLOSED_RAMP_RATE)
            .withClosedLoopController(
                TurretConstants.ClosedLoopController.P,
                TurretConstants.ClosedLoopController.I,
                TurretConstants.ClosedLoopController.D,
                TurretConstants.Velocity.MAX_VELOCITY,
                TurretConstants.Velocity.ACCELERATION)
            .withSimClosedLoopController(
                TurretConstants.ClosedLoopController.SIM.P,
                TurretConstants.ClosedLoopController.SIM.I,
                TurretConstants.ClosedLoopController.SIM.D,
                TurretConstants.Velocity.MAX_VELOCITY,
                TurretConstants.Velocity.ACCELERATION)
            .withFeedforward(
                new SimpleMotorFeedforward(
                    TurretConstants.FeedForward.KS,
                    TurretConstants.FeedForward.KV,
                    TurretConstants.FeedForward.KA))
            .withSimFeedforward(
                new SimpleMotorFeedforward(
                    TurretConstants.FeedForward.SIM.KS,
                    TurretConstants.FeedForward.SIM.KV,
                    TurretConstants.FeedForward.SIM.KA))
            .withControlMode(TurretConstants.CONTROL_MODE);

    this.motor = new SparkFlex(TurretConstants.TURRET_MOTOR_ID, TurretConstants.MOTOR_TYPE);

    this.smartMotor = new SparkWrapper(this.motor, TurretConstants.TURRET_MOTOR, this.motorConfig);

    this.mechanismConfig =
        new PivotConfig(smartMotor)
            .withStartingPosition(TurretConstants.Position.STARTING_POSITION)
            .withHardLimit(
                TurretConstants.Position.TURRET_MIN_ANGLE,
                TurretConstants.Position.TURRET_MAX_ANGLE)
            .withMOI(TurretConstants.TURRET_SIZE, TurretConstants.TURRET_MASS);

    this.mechanism = new Pivot(mechanismConfig);

    this.inputs = new TurretInputsAutoLogged();
  }

  public static TurretSubsystem getInstance() {
    if (instance == null) {
      instance = new TurretSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    ShooterCalculator.update();
    this.mechanism.updateTelemetry();
    this.updateInputs();
  }

  @Override
  public void simulationPeriodic() {
    this.mechanism.simIterate();
  }

  private void updateInputs() {
    this.inputs.currentAngle = getCurrentAngleDegrees();
    this.inputs.targetAngle = ShooterCalculator.getTurretTargetAngleValue();
    this.inputs.atTarget = isAtTargetAngle(ShooterCalculator.getTurretTargetAngleValue());
    this.inputs.distanceToTarget = ShooterCalculator.getDistanceToTargetValue();
    this.inputs.fieldAngleToTarget = ShooterCalculator.getFieldAngleToTargetValue();

    Logger.processInputs("Subsystems/Shooter/Turret/", inputs);
  }

  public void setTargetAngle(double angleDegrees) {
    this.smartMotor.setPosition(Angle.ofBaseUnits(angleDegrees, Degrees));
  }

  public void setTargetAngle(Angle angle) {
    setTargetAngle(angle.in(Degrees));
  }

  public void stop() {
    this.smartMotor.setDutyCycle(0);
  }

  public double getCurrentAngleDegrees() {
    return this.smartMotor.getMechanismPosition().in(Degrees);
  }

  public Angle getCurrentAngle() {
    return this.smartMotor.getMechanismPosition();
  }

  public boolean isAtTargetAngle(double targetAngleDegrees) {
    return this.smartMotor.getMechanismPosition().in(Degrees)
            <= HoodConstants.Position.ANGLE_ERROR_ALLOWED + targetAngleDegrees
        && this.smartMotor.getMechanismPosition().in(Degrees)
            >= -HoodConstants.Position.ANGLE_ERROR_ALLOWED + targetAngleDegrees;
  }

  public double getMinAngleDegrees() {
    return TurretConstants.Position.TURRET_MIN_ANGLE.in(Degrees);
  }

  public double getMaxAngleDegrees() {
    return TurretConstants.Position.TURRET_MAX_ANGLE.in(Degrees);
  }
}
