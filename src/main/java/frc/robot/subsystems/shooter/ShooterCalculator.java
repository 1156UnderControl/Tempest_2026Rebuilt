package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.java_is_undercontrol.util.AllianceFlipUtil;
import frc.robot.Constants;
import frc.robot.RobotStates;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.FieldStructurePoses;
import frc.robot.subsystems.shooter.interpolatingTreeMaps.FeedingMaps;
import frc.robot.subsystems.shooter.interpolatingTreeMaps.InterpolatingTreeMaps;
import frc.robot.subsystems.shooter.interpolatingTreeMaps.ShootingMaps;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterCalculator {
  private static ShooterCalculator instance;

  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

  private Rotation2d lastTurretAngle;
  private double lastHoodAngle;
  private Rotation2d turretAngle;
  private double hoodAngle = Double.NaN;
  private double turretVelocity;
  private double hoodVelocity;

  public static ShooterCalculator getInstance() {
    if (instance == null) instance = new ShooterCalculator();
    return instance;
  }

  public record ShootingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  // Cache parameters
  private ShootingParameters latestParameters = null;

  private boolean lastState;

  private final Transform2d robotToTurret = TurretConstants.TURRET_OFFSET_FROM_ROBOT_CENTER;

  private static double phaseDelay = TurretConstants.PHASE_DELAY;

  @AutoLogOutput private double distanceToTarget = 0;

  public ShootingParameters getParameters() {
    return this.getParameters(true);
  }

  public ShootingParameters getLastParameters() {
    return latestParameters;
  }

  public ShootingParameters getParameters(boolean shooting) {
    if (latestParameters != null && lastState == shooting) {
      return latestParameters;
    }

    lastState = shooting;

    InterpolatingTreeMaps maps = getMaps(shooting);

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = RobotStates.getInstance().getRobotPose();
    ChassisSpeeds robotRelativeVelocity = RobotStates.getInstance().getRobotVelocity();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));
    // Calculate distance from turret to target
    Translation2d target = getTargetTranslation2d(shooting);
    Logger.recordOutput("target", target);
    Pose2d turretPosition = estimatedPose.transformBy(robotToTurret);
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = RobotStates.getInstance().getFieldVelocity();
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getY() * Math.cos(robotAngle)
                    - robotToTurret.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getX() * Math.cos(robotAngle)
                    - robotToTurret.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = maps.getTimeOfFlightMap().get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
      distanceToTarget = lookaheadTurretToTargetDistance;
    }

    // Calculate parameters accounted for imparted velocity
    turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    hoodAngle = maps.getHoodAngleMap().get(lookaheadTurretToTargetDistance).getRadians();
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    turretVelocity =
        turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRadians() / Constants.loopPeriodSecs);
    hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;

    Logger.recordOutput("LookAheadPose", lookaheadPose);

    latestParameters =
        new ShootingParameters(
            lookaheadTurretToTargetDistance >= maps.getMinDistance()
                && lookaheadTurretToTargetDistance <= maps.getMaxDistance(),
            turretAngle.minus(Rotation2d.fromRadians(robotAngle)),
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            maps.getFlywheelSpeedMap().get(lookaheadTurretToTargetDistance));

    return latestParameters;
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }

  private Translation2d getTargetTranslation2d(boolean shooting) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    if (shooting) {
      return alliance == Alliance.Blue
          ? FieldStructurePoses.BLUE_HUB_POSE
          : FieldStructurePoses.RED_HUB_POSE;
    }

    if (RobotStates.getInstance().isOnBlueDepotSide()) {
      return AllianceFlipUtil.applyOnlyX(FieldConstants.FieldStrategicPoses.LEFT_FEED_POSE);
    } else {
      return AllianceFlipUtil.applyOnlyX(FieldConstants.FieldStrategicPoses.RIGHT_FEED_POSE);
    }
  }

  private InterpolatingTreeMaps getMaps(boolean shooting) {
    if (shooting) return new ShootingMaps();
    return new FeedingMaps();
  }
}
