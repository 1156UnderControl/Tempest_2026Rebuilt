package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
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
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class TurretSubsystem extends SubsystemBase {

  private final ShooterCalculator calculator;
  private final TurretInputsAutoLogged inputs;
  private static TurretSubsystem instance;

  private final SparkFlex turretFlex;
  private final SmartMotorControllerConfig smartTurretMotorConfig;
  private final SmartMotorController turretMotor;
  private final PivotConfig turretMechanismConfig;
  private final Pivot turret;

  private TurretSubsystem() {
    calculator = ShooterCalculator.getInstance();
    smartTurretMotorConfig =
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

    turretFlex = new SparkFlex(TurretConstants.TURRET_MOTOR_ID, TurretConstants.MOTOR_TYPE);

    turretMotor =
        new SparkWrapper(turretFlex, TurretConstants.TURRET_MOTOR, smartTurretMotorConfig);

    turretMechanismConfig =
        new PivotConfig(turretMotor)
            .withStartingPosition(TurretConstants.Position.STARTING_POSITION)
            .withHardLimit(
                TurretConstants.Position.TURRET_MIN_ANGLE,
                TurretConstants.Position.TURRET_MAX_ANGLE)
            .withMOI(TurretConstants.TURRET_SIZE, TurretConstants.TURRET_MASS)
            .withTelemetry("Turret", TurretConstants.TELEMETRY);

    turret = new Pivot(turretMechanismConfig);

    SparkAbsoluteEncoder encoder1 = turretFlex.getAbsoluteEncoder();
    // TODO: define second encoder correctly
    SparkAbsoluteEncoder encoder2 = turretFlex.getAbsoluteEncoder();

    EasyCRTConfig easyCRTConfig =
        new EasyCRTConfig(
                () -> Angle.ofBaseUnits(encoder1.getPosition(), Rotations),
                () -> Angle.ofBaseUnits(encoder2.getPosition(), Rotations))
            .withEncoderRatios(
                TurretConstants.TURRET_GEAR_TEETH / TurretConstants.ENCODER1_GEAR_TEETH,
                TurretConstants.TURRET_GEAR_TEETH / TurretConstants.ENCODER2_GEAR_TEETH)
            .withMechanismRange(
                TurretConstants.Position.TURRET_MIN_ANGLE,
                TurretConstants.Position.TURRET_MAX_ANGLE);

    EasyCRT easyCRT = new EasyCRT(easyCRTConfig);

    // easyCRT
    //     .getAngleOptional()
    //     .ifPresent(
    //         mechAngle -> {
    //           turretSmartMotor.setEncoderPosition(mechAngle);
    //         });

    inputs = new TurretInputsAutoLogged();
  }

  public static TurretSubsystem getInstance() {
    if (instance == null) {
      instance = new TurretSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    turret.updateTelemetry();
    updateInputs();
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }

  private void updateInputs() {
    inputs.currentAngle = turret.getAngle().in(Degrees);
    inputs.targetAngle = calculator.getParameters().turretAngle().getDegrees();
    inputs.atTarget = isAtTargetAngle();

    Logger.processInputs("Subsystems/Shooter/Turret/", inputs);
  }

  public Command trackHub() {
    return turret.setAngle(
        () -> Degrees.of(calculator.getParameters().turretAngle().getDegrees()));
  }

  public Command trackPass() {
    return turret.setAngle(
        () -> Degrees.of(calculator.getParameters(false).turretAngle().getDegrees()));
  }

  public boolean isAtTargetAngle() {
    return turret
        .getAngle()
        .isNear(
            turret.getMechanismSetpoint().isEmpty()
                ? Degrees.of(0)
                : turret.getAngle(),
            TurretConstants.Position.ANGLE_ERROR_ALLOWED);
  }
}
