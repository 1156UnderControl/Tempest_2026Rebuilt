package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    this.turretSmartMotor =
        new SparkWrapper(this.turretMotor, TurretConstants.TURRET_MOTOR, this.turretMotorConfig);

    this.turretMechanismConfig =
        new PivotConfig(turretSmartMotor)
            .withStartingPosition(TurretConstants.Position.STARTING_POSITION)
            .withHardLimit(
                TurretConstants.Position.TURRET_MIN_ANGLE,
                TurretConstants.Position.TURRET_MAX_ANGLE)
            .withMOI(TurretConstants.TURRET_SIZE, TurretConstants.TURRET_MASS)
            .withTelemetry("Turret", TurretConstants.TELEMETRY);

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
    this.inputs.currentAngle = this.turretMechanism.getAngle().in(Degrees);
    this.inputs.targetAngle = this.calculator.getParameters().turretAngle().getDegrees();
    this.inputs.atTarget = this.isAtTargetAngle();

    Logger.processInputs("Subsystems/Shooter/Turret/", inputs);
  }

  public Command trackHub() {
    return this.turretMechanism.setAngle(
        () -> Degrees.of(this.calculator.getParameters().turretAngle().getDegrees()));
  }

  public boolean isAtTargetAngle() {
    return this.turretMechanism
        .getAngle()
        .isNear(
            this.turretMechanism.getMechanismSetpoint().isEmpty()
                ? Degrees.of(0)
                : this.turretMechanism.getMechanismSetpoint().get(),
            TurretConstants.Position.ANGLE_ERROR_ALLOWED);
  }
}
