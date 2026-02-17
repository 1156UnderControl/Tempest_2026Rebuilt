package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStates;
import frc.robot.subsystems.shooter.ShooterCalculator;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

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

  private double absoluteAngle;

  private TurretSubsystem(SparkAbsoluteEncoder kickerEncoder, SparkAbsoluteEncoder climberEncoder) {
    calculator = ShooterCalculator.getInstance();
    smartTurretMotorConfig =
        new SmartMotorControllerConfig(this)
            .withTelemetry("Motor Turret", TelemetryVerbosity.HIGH)
            .withIdleMode(TurretConstants.MOTOR_MODE)
            .withGearing(TurretConstants.GEARING)
            .withMotorInverted(TurretConstants.INVERTED)
            .withSoftLimit(TurretConstants.TURRET_MIN_ANGLE, TurretConstants.TURRET_MAX_ANGLE)
            .withStatorCurrentLimit(TurretConstants.STATOR_CURRENT_LIMIT)
            .withClosedLoopRampRate(TurretConstants.CLOSED_RAMP_RATE)
            .withClosedLoopController(
                TurretConstants.FeedBackController.kP,
                TurretConstants.FeedBackController.kI,
                TurretConstants.FeedBackController.kD,
                TurretConstants.MAX_VELOCITY,
                TurretConstants.MAX_ACCELERATION)
            .withSimClosedLoopController(
                TurretConstants.FeedBackController.SIM.kP,
                TurretConstants.FeedBackController.SIM.kI,
                TurretConstants.FeedBackController.SIM.kD,
                TurretConstants.MAX_VELOCITY,
                TurretConstants.MAX_ACCELERATION)
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
            .withControlMode(TurretConstants.CONTROL_MODE)
            .withStartingPosition(TurretConstants.STARTING_ANGLE);

    turretFlex = new SparkFlex(TurretConstants.TURRET_MOTOR_ID, TurretConstants.MOTOR_TYPE);

    turretMotor = new SparkWrapper(turretFlex, TurretConstants.DC_MOTOR, smartTurretMotorConfig);

    turretMechanismConfig =
        new PivotConfig(turretMotor)
            .withHardLimit(TurretConstants.TURRET_MIN_ANGLE, TurretConstants.TURRET_MAX_ANGLE)
            .withMOI(TurretConstants.TURRET_SIZE, TurretConstants.TURRET_MASS)
            .withTelemetry(TurretConstants.TURRET_MECHANISM_NAME, TurretConstants.TELEMETRY);

    turret = new Pivot(turretMechanismConfig);

    this.encoder1 = kickerEncoder;
    this.encoder2 = climberEncoder;

    Logger.recordOutput("EncodersCRT/encoder1", this.encoder1.getPosition());
    Logger.recordOutput("EncodersCRT/encoder2", this.encoder2.getPosition());
    Logger.recordOutput(
        "EncodersCRT/absoluteResult",
        determineTurretAngleDegrees(
            Units.rotationsToDegrees(this.encoder1.getPosition()),
            Units.rotationsToDegrees(this.encoder2.getPosition())));

    // this.absoluteAngle =
    // determineTurretAngleDegrees(
    // Units.rotationsToDegrees(this.encoder1.getPosition()),
    // Units.rotationsToDegrees(this.encoder2.getPosition()));

    // this.turretMotor.setEncoderPosition(Degrees.of(this.absoluteAngle));

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
        inputs.targetAngle =
            turret.getMechanismSetpoint().isEmpty()
                ? 0
                : turret.getMechanismSetpoint().get().in(Degrees);
    inputs.atTarget = RobotStates.getInstance().isOnAllianceZone() ? isAtTargetAngleShootHub() : isAtTargetAngleShootPass();
    inputs.current = turretMotor.getStatorCurrent();
    inputs.voltage = turretMotor.getVoltage();
    inputs.turret =
        RobotStates.getInstance()
            .getRobotPose()
            .transformBy(TurretConstants.TURRET_OFFSET_FROM_ROBOT_CENTER)
            .plus(
                new Transform2d(
                    Translation2d.kZero, Rotation2d.fromDegrees(turret.getAngle().in(Degrees))));
    inputs.absoluteAngle = this.absoluteAngle;
    inputs.encoder1 = this.encoder1.getPosition();
    inputs.encoder2 = this.encoder2.getPosition();

    Logger.processInputs("Subsystems/Shooter/Turret/", inputs);
  }

  public Command trackHub() {
    return turret.setAngle(
        () ->
            Degrees.of(
                getClosestTargetAngle(
                    calculator.getParameters(true).turretAngle().getDegrees(),
                    turret.getAngle().in(Degrees))));
  }

  public Command trackPass() {
    return turret.setAngle(
        () ->
            Degrees.of(
                getClosestTargetAngle(
                    calculator.getParameters(false).turretAngle().getDegrees(),
                    turret.getAngle().in(Degrees))));
  }

  public Command trackNonOdometry() {
    return turret.setAngle(Degrees.of(getClosestTargetAngle(180, turret.getAngle().in(Degrees))));
  }

  public boolean isAtTargetAngleShootHub() {
    return turret
        .getAngle()
        .isNear(
            Degrees.of(calculator.getParameters(true).turretAngle().getDegrees()),
            TurretConstants.ANGLE_ERROR_ALLOWED);
  }

  public boolean isAtTargetAngleShootPass() {
    return turret
        .getAngle()
        .isNear(
            Degrees.of(calculator.getParameters(false).turretAngle().getDegrees()),
            TurretConstants.ANGLE_ERROR_ALLOWED);
  }

  public Command resetCRT() {
    return Commands.runOnce(
        () -> {
          Logger.recordOutput("EncodersCRT/encoder1", this.encoder1.getPosition());
          Logger.recordOutput("EncodersCRT/encoder2", this.encoder2.getPosition());
          Logger.recordOutput(
              "EncodersCRT/absoluteResult",
              determineTurretAngleDegrees(
                  Units.rotationsToDegrees(this.encoder1.getPosition()),
                  Units.rotationsToDegrees(this.encoder2.getPosition())));
        });
  }

  private double getClosestTargetAngle(double targetAngle, double currentAngle) {
    double delta = (targetAngle - currentAngle) % 360;

    if (delta > 180) {
      delta -= 360;
    } else if (delta < -180) {
      delta += 360;
    }

    double potentialTarget = currentAngle + delta;

    if (potentialTarget > TurretConstants.TURRET_MAX_ANGLE.in(Degrees)) {
      potentialTarget -= 360;
    } else if (potentialTarget < TurretConstants.TURRET_MIN_ANGLE.in(Degrees)) {
      potentialTarget += 360;
    }

    return potentialTarget;
  }

  private double determineTurretAngleDegrees(double e1, double e2) {
    double difference = e2 - e1;
    if (difference > 250) {
      difference -= 360;
    }
    if (difference < -250) {
      difference += 360;
    }
    difference *= TurretConstants.SLOPE;

    double e1Rotations =
        (difference * TurretConstants.GEAR_0_TOOTH_COUNT / TurretConstants.GEAR_1_TOOTH_COUNT)
            / 360.0;
    double e1RotationsFloored = Math.floor(e1Rotations);
    double turretAngle =
        (e1RotationsFloored * 360.0 + e1)
            * (TurretConstants.GEAR_1_TOOTH_COUNT / TurretConstants.GEAR_0_TOOTH_COUNT);
    if (turretAngle - difference < -100) {
      turretAngle +=
          TurretConstants.GEAR_1_TOOTH_COUNT / TurretConstants.GEAR_0_TOOTH_COUNT * 360.0;
    } else if (turretAngle - difference > 100) {
      turretAngle -=
          TurretConstants.GEAR_1_TOOTH_COUNT / TurretConstants.GEAR_0_TOOTH_COUNT * 360.0;
    }
    return -turretAngle + 180;
  }
}
