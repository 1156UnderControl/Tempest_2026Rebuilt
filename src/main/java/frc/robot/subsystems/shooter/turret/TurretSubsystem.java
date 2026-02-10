package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStates;
import frc.robot.subsystems.shooter.ShooterCalculator;
import org.littletonrobotics.junction.Logger;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
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

  private final SparkAbsoluteEncoder encoder1;
  private final SparkAbsoluteEncoder encoder2;

  private final EasyCRT easyCRT;

  private double absoluteAngle;

  private TurretSubsystem(SparkAbsoluteEncoder kickerEncoder, SparkAbsoluteEncoder climberEncoder) {
    this.absoluteAngle = 2;
    calculator = ShooterCalculator.getInstance();
    smartTurretMotorConfig =
        new SmartMotorControllerConfig(this)
            .withIdleMode(TurretConstants.MOTOR_MODE)
            .withGearing(TurretConstants.GEARING)
            .withMotorInverted(TurretConstants.INVERTED)
            .withSoftLimit(TurretConstants.TURRET_MIN_ANGLE, TurretConstants.TURRET_MAX_ANGLE)
            .withStatorCurrentLimit(TurretConstants.STATOR_CURRENT_LIMIT)
            .withClosedLoopRampRate(TurretConstants.CLOSED_RAMP_RATE)
            .withClosedLoopController(
                new ExponentialProfilePIDController(
                    new PIDController(
                        TurretConstants.FeedBackController.kP,
                        TurretConstants.FeedBackController.kI,
                        TurretConstants.FeedBackController.kD),
                    ExponentialProfilePIDController.createConstraints(
                        TurretConstants.MAX_VOLTS,
                        TurretConstants.MAX_VELOCITY,
                        TurretConstants.ACCELERATION)))
            .withSimClosedLoopController(
                new ExponentialProfilePIDController(
                    new PIDController(
                        TurretConstants.FeedBackController.SIM.kP,
                        TurretConstants.FeedBackController.SIM.kI,
                        TurretConstants.FeedBackController.SIM.kD),
                    ExponentialProfilePIDController.createConstraints(
                        TurretConstants.MAX_VOLTS,
                        TurretConstants.MAX_VELOCITY,
                        TurretConstants.ACCELERATION)))
            .withFeedforward(
                new SimpleMotorFeedforward(
                    TurretConstants.FeedForwardController.kS,
                    TurretConstants.FeedForwardController.kV,
                    TurretConstants.FeedForwardController.kA))
            .withSimFeedforward(
                new SimpleMotorFeedforward(
                    TurretConstants.FeedForwardController.SIM.kS,
                    TurretConstants.FeedForwardController.SIM.kV,
                    TurretConstants.FeedForwardController.SIM.kA))
            .withControlMode(TurretConstants.CONTROL_MODE);

    turretFlex = new SparkFlex(TurretConstants.TURRET_MOTOR_ID, TurretConstants.MOTOR_TYPE);

    turretMotor = new SparkWrapper(turretFlex, TurretConstants.DC_MOTOR, smartTurretMotorConfig);

    turretMechanismConfig =
        new PivotConfig(turretMotor)
            .withHardLimit(TurretConstants.TURRET_MIN_ANGLE, TurretConstants.TURRET_MAX_ANGLE)
            .withSoftLimits(TurretConstants.TURRET_MIN_ANGLE, TurretConstants.TURRET_MAX_ANGLE)
            .withMOI(TurretConstants.TURRET_SIZE, TurretConstants.TURRET_MASS)
            .withTelemetry(TurretConstants.TURRET_MECHANISM_NAME, TurretConstants.TELEMETRY);

    turret = new Pivot(turretMechanismConfig);

    encoder1 = kickerEncoder;
    // TODO: define second encoder correctly
    encoder2 = climberEncoder;

    EasyCRTConfig easyCRTConfig =
        new EasyCRTConfig(
                () -> Angle.ofBaseUnits(encoder1.getPosition(), Rotations),
                () -> Angle.ofBaseUnits(encoder2.getPosition(), Rotations))
            .withEncoderRatios(
                TurretConstants.TURRET_GEAR_TEETH / 14, TurretConstants.TURRET_GEAR_TEETH / 15)
            .withMechanismRange(TurretConstants.TURRET_MIN_ANGLE, TurretConstants.TURRET_MAX_ANGLE);

    this.easyCRT = new EasyCRT(easyCRTConfig);

    this.easyCRT
        .getAngleOptional()
        .ifPresent(
            mechAngle -> {
              turretMotor.setEncoderPosition(mechAngle);
              this.absoluteAngle = mechAngle.in(Degrees);
            });

    inputs = new TurretInputsAutoLogged();
  }

  public static TurretSubsystem getInstance(
      SparkAbsoluteEncoder kickerEncoder, SparkAbsoluteEncoder climberEncoder) {
    if (instance == null) {
      instance = new TurretSubsystem(kickerEncoder, climberEncoder);
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
    inputs.targetAngle =
        calculator.getLastParameters() != null
            ? calculator.getLastParameters().turretAngle().getDegrees()
            : 0;
    inputs.atTarget = isAtTargetAngle();
    inputs.turret =
        RobotStates.getInstance()
            .getRobotPose()
            .transformBy(TurretConstants.TURRET_OFFSET_FROM_ROBOT_CENTER)
            .plus(
                new Transform2d(
                    Translation2d.kZero, Rotation2d.fromDegrees(turret.getAngle().in(Degrees))));

    Logger.processInputs("Subsystems/Shooter/Turret/", inputs);
  }

  public Command trackHub() {
    return turret.setAngle(
        () ->
            Degrees.of(
                getReachableTurretAngle(calculator.getParameters().turretAngle().getDegrees())));
  }

  public Command trackPass() {
    return turret.setAngle(
        () ->
            Degrees.of(
                getReachableTurretAngle(
                    calculator.getParameters(false).turretAngle().getDegrees())));
  }

  public boolean isAtTargetAngle() {
    return turret
        .getAngle()
        .isNear(
            turret.getMechanismSetpoint().isEmpty() ? Degrees.of(0) : turret.getAngle(),
            TurretConstants.ANGLE_ERROR_ALLOWED);
  }

  private double getReachableTurretAngle(double targetAngle) {
    if (targetAngle > TurretConstants.TURRET_MAX_ANGLE.in(Degrees)) {
      return targetAngle - 360;
    } else if (targetAngle < TurretConstants.TURRET_MIN_ANGLE.in(Degrees)) {
      return targetAngle + 360;
    } else {
      return targetAngle;
    }
  }
}
