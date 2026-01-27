package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.shooter.HoodConstants;
import frc.robot.constants.shooter.TurretConstants;
import frc.robot.subsystems.shooter.ShooterCalculator;
import org.littletonrobotics.junction.Logger;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class TurretSubsystem extends SubsystemBase {

  private final ShooterCalculator calculator;

  private final SmartMotorControllerConfig turretMotorConfig;
  private final SparkFlex turretMotor;
  private final SmartMotorController turretSmartMotor;
  private final PivotConfig turretMechanismConfig;
  private final Pivot turretMechanism;

  private final TurretInputsAutoLogged inputs;

  private static TurretSubsystem instance;

  private TurretSubsystem() {
    this.calculator = ShooterCalculator.getInstance();
    this.turretMotorConfig =
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
                new ExponentialProfilePIDController(
                    new PIDController(
                        TurretConstants.ClosedLoopController.kP,
                        TurretConstants.ClosedLoopController.kI,
                        TurretConstants.ClosedLoopController.kD),
                    ExponentialProfilePIDController.createConstraints(
                        TurretConstants.MAX_VOLTS,
                        TurretConstants.Velocity.MAX_VELOCITY,
                        TurretConstants.Velocity.ACCELERATION)))
            .withSimClosedLoopController(
                new ExponentialProfilePIDController(
                    new PIDController(
                        TurretConstants.ClosedLoopController.kP,
                        TurretConstants.ClosedLoopController.kI,
                        TurretConstants.ClosedLoopController.kD),
                    ExponentialProfilePIDController.createConstraints(
                        TurretConstants.MAX_VOLTS,
                        TurretConstants.Velocity.MAX_VELOCITY,
                        TurretConstants.Velocity.ACCELERATION)))
            .withFeedforward(
                new SimpleMotorFeedforward(
                    TurretConstants.FeedForward.kS,
                    TurretConstants.FeedForward.kV,
                    TurretConstants.FeedForward.kA))
            .withSimFeedforward(
                new SimpleMotorFeedforward(
                    TurretConstants.FeedForward.SIM.kS,
                    TurretConstants.FeedForward.SIM.kV,
                    TurretConstants.FeedForward.SIM.kA))
            .withControlMode(TurretConstants.CONTROL_MODE);

    this.turretMotor = new SparkFlex(TurretConstants.TURRET_MOTOR_ID, TurretConstants.MOTOR_TYPE);

    this.turretSmartMotor = new SparkWrapper(this.turretMotor, TurretConstants.TURRET_MOTOR, this.turretMotorConfig);

    this.turretMechanismConfig =
        new PivotConfig(turretSmartMotor)
            .withStartingPosition(TurretConstants.Position.STARTING_POSITION)
            .withHardLimit(
                TurretConstants.Position.TURRET_MIN_ANGLE,
                TurretConstants.Position.TURRET_MAX_ANGLE)
            .withMOI(TurretConstants.TURRET_SIZE, TurretConstants.TURRET_MASS)
            .withTelemetry(TurretConstants.MOTOR_NAME, TurretConstants.TELEMETRY);

    this.turretMechanism = new Pivot(turretMechanismConfig);

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
    this.turretMechanism.updateTelemetry();
    this.updateInputs();
  }

  @Override
  public void simulationPeriodic() {
    this.turretMechanism.simIterate();
  }

  private void updateInputs() {
    this.inputs.currentAngle = this.turretSmartMotor.getMechanismPosition().in(Degrees);
    this.inputs.targetAngle = this.calculator.getParameters().turretTargetAngle();
    this.inputs.distanceToTarget = this.calculator.getParameters().distanceToTarget();
    this.inputs.atTarget = this.isAtTargetAngle();

    Logger.processInputs("Subsystems/Shooter/Turret/", inputs);
  }

  public void goToTargetAngle() {
    this.turretSmartMotor.setPosition(Angle.ofBaseUnits(this.calculator.getParameters().turretTargetAngle(), Degrees));
  }

  public boolean isAtTargetAngle() {
    return this.turretSmartMotor.getMechanismPosition().in(Degrees)
            <= HoodConstants.Position.ANGLE_ERROR_ALLOWED + (this.turretSmartMotor.getMechanismPositionSetpoint().isEmpty() == true ? 0 : this.turretSmartMotor.getMechanismPositionSetpoint().get().in(Degrees))
        && this.turretSmartMotor.getMechanismPosition().in(Degrees)
            >= -HoodConstants.Position.ANGLE_ERROR_ALLOWED + (this.turretSmartMotor.getMechanismPositionSetpoint().isEmpty() == true ? 0 : this.turretSmartMotor.getMechanismPositionSetpoint().get().in(Degrees));
  }
}
