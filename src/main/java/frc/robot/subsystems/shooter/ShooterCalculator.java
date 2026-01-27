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
  private static ShooterCalculator instance;

  private static boolean isHubMode;

  private static ShooterParameters latestParameters;

  private ShooterCalculator(){
    isHubMode = true;
  }

  public static ShooterCalculator getInstance() {
    if (instance == null) instance = new ShooterCalculator();
      return instance;
  }

  public ShooterParameters getParameters() {
    if (latestParameters != null) {
      return latestParameters;
    }
    latestParameters =
        new ShooterParameters(
            getTurretTargetAngle(),
            getHoodTargetAngle(),
            getFlyWheelsTargetVelocity(),
            getDistanceToTargetPose());

    return latestParameters;
  }

  public void clearShooterParameters() {
    latestParameters = null;
  }

  public void setHubMode(boolean hubMode) {
    isHubMode = hubMode;
  }

  public boolean isHubMode() {
    return isHubMode;
  }

  private double getTurretTargetAngle() {
    double alphaAngle = getFieldAngleToTarget();
    double robotHeading = RobotStates.getInstance().getRobotPose().getRotation().getDegrees();
    double turretTargetAngle = alphaAngle - robotHeading;

    return getReacheableTurretAngle(turretTargetAngle);
  }

  private static double getHoodTargetAngle() {
    return 45;
  }

  private static double getFlyWheelsTargetVelocity() {
    return 0.5;
  }

  private static double getDistanceToTargetPose() {
    return Math.hypot(getXSideFromTriangle(), getYSideFromTriangle());
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

  private static double getFieldAngleToTarget() {
    double alphaAngle = Math.atan2(getYSideFromTriangle(), getXSideFromTriangle());
    return Math.toDegrees(alphaAngle);
  }

  private static Pose2d getTargetAlliancePose() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue); // valor padrão

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
    Translation2d rotatedOffset = turretOffset.rotateBy(RobotStates.getInstance().getRobotPose().getRotation());
    return new Pose2d(
        RobotStates.getInstance().getRobotPose().getTranslation().plus(rotatedOffset),
        RobotStates.getInstance().getRobotPose().getRotation());
  }

  private static double getXSideFromTriangle() {
    return getTargetAlliancePose().getX()
        - getShooterFieldPoseFromRobotCenter(TurretConstants.Position.TURRET_OFFSET_FROM_ROBOT_CENTER)
            .getX();
  }

  private static double getYSideFromTriangle() {
    return getTargetAlliancePose().getY()
        - getShooterFieldPoseFromRobotCenter(TurretConstants.Position.TURRET_OFFSET_FROM_ROBOT_CENTER)
            .getY();
  }
}
