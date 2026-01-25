package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotStates;
import frc.robot.constants.FieldConstants.FieldStructurePoses;
import frc.robot.constants.shooter.TurretConstants;

public class ShooterCalculator {

  private static boolean isHubMode = true;

  private static double turretTargetAngle = 0;
  private static double hoodTargetAngle = 0;
  private static double flyWheelsTargetVelocity = 0;
  private static double distanceToTarget = 0;
  private static double fieldAngleToTarget = 0;

  private ShooterCalculator() {}

  public static void update() {
    distanceToTarget = getDistanceToTargetPose();
    fieldAngleToTarget = getFieldAngleToTarget();
    turretTargetAngle = getTurretTargetAngle();
    hoodTargetAngle = getHoodTargetAngle(distanceToTarget);
    flyWheelsTargetVelocity = getFlyWheelsTargetVelocity(distanceToTarget);
  }

  public static void setHubMode(boolean hubMode) {
    isHubMode = hubMode;
  }

  public static boolean isHubMode() {
    return isHubMode;
  }

  public static double getTurretTargetAngleValue() {
    return turretTargetAngle;
  }

  public static double getHoodTargetAngleValue() {
    return hoodTargetAngle;
  }

  public static double getFlyWheelsTargetVelocityValue() {
    return flyWheelsTargetVelocity;
  }

  public static double getDistanceToTargetValue() {
    return distanceToTarget;
  }

  public static double getFieldAngleToTargetValue() {
    return fieldAngleToTarget;
  }

  public static Pose2d getShooterFieldPose() {
    return getShooterFieldPoseFromRobotCenter(
        TurretConstants.Position.TURRET_OFFSET_FROM_ROBOT_CENTER);
  }

  private static double getTurretTargetAngle() {
    double alphaAngle = getFieldAngleToTarget();
    double robotHeading = RobotStates.getInstance().getRobotPose().getRotation().getDegrees();
    double turretTarget = alphaAngle - robotHeading;

    return getReacheableTurretAngle(turretTarget);
  }

  private static double getReacheableTurretAngle(double targetAngle) {
    if (targetAngle > TurretConstants.Position.TURRET_MAX_ANGLE.in(Degrees)) {
      return targetAngle - 400;
    } else if (targetAngle < TurretConstants.Position.TURRET_MIN_ANGLE.in(Degrees)) {
      return targetAngle + 400;
    } else {
      return targetAngle;
    }
  }

  private static double getFlyWheelsTargetVelocity(double distanceToTargetPose) {
    return 0.5;
  }

  private static double getHoodTargetAngle(double distanceToTargetPose) {
    return 45;
  }

  private static double getDistanceToTargetPose() {
    return Math.hypot(getXSideFromTriangle(), getYSideFromTriangle());
  }

  private static double getFieldAngleToTarget() {
    double alphaAngle = Math.atan2(getYSideFromTriangle(), getXSideFromTriangle());
    return Math.toDegrees(alphaAngle);
  }

  private static double getXSideFromTriangle() {
    return getTargetAlliancePose().getX()
        - getShooterFieldPoseFromRobotCenter(
                TurretConstants.Position.TURRET_OFFSET_FROM_ROBOT_CENTER)
            .getX();
  }

  private static double getYSideFromTriangle() {
    return getTargetAlliancePose().getY()
        - getShooterFieldPoseFromRobotCenter(
                TurretConstants.Position.TURRET_OFFSET_FROM_ROBOT_CENTER)
            .getY();
  }

  private static Pose2d getTargetAlliancePose() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    if (isHubMode) {
      return alliance == Alliance.Blue
          ? FieldStructurePoses.BLUE_HUB_POSE
          : FieldStructurePoses.RED_HUB_POSE;
    } else {
      return alliance == Alliance.Blue
          ? FieldStructurePoses.BLUE_HUB_POSE
          : FieldStructurePoses.RED_HUB_POSE;
    }
  }

  private static Pose2d getShooterFieldPoseFromRobotCenter(Translation2d turretOffset) {
    Translation2d rotatedOffset =
        turretOffset.rotateBy(RobotStates.getInstance().getRobotPose().getRotation());
    return new Pose2d(
        RobotStates.getInstance().getRobotPose().getTranslation().plus(rotatedOffset),
        RobotStates.getInstance().getRobotPose().getRotation());
  }
}
