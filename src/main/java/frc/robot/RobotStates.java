package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.java_is_undercontrol.util.AllianceFlipUtil;
import frc.robot.constants.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class RobotStates {

  private static Pose2d robotPose = Pose2d.kZero;
  private static ChassisSpeeds robotVelocity = new ChassisSpeeds();

  private static RobotStates instance;

  private static boolean isNonOdometryMode = false;

  private static boolean extendedMode = false;

  private static boolean shootingMode = false;

  private static boolean shouldShootAutonomous = false;

  public static RobotStates getInstance() {
    if (instance == null) {
      instance = new RobotStates();
    }
    return instance;
  }

  public Rotation2d getRobotRotation() {
    return robotPose.getRotation();
  }

  public Pose2d getRobotPose() {
    return robotPose;
  }

  public void setRobotPose(Pose2d newPose) {
    robotPose = newPose;
  }

  public void setRobotVelocity(ChassisSpeeds newChassisSpeeds) {
    robotVelocity = newChassisSpeeds;
  }

  public ChassisSpeeds getRobotVelocity() {
    return robotVelocity;
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRobotRotation());
  }


  public boolean isOnBlueDepotSide() {
    return (robotPose.getY() > FieldConstants.fieldWidth / 2);
  }

  public boolean isOnRedDepotSide() {
    return (robotPose.getY() < FieldConstants.fieldWidth / 2);
  }

  public boolean isOnAllianceZone() {
    return AllianceFlipUtil.apply(robotPose).getX()
        < FieldConstants.FieldStrategicPoses.AllianceZoneX;
  }

  public boolean isOnOtherAllianceZone() {
    return AllianceFlipUtil.apply(robotPose).getX()
        > FieldConstants.FieldStrategicPoses.OtherAllianceZoneX;
  }

  public boolean isOnNotShootZone() {
    if (!isOnAllianceZone()
        && (isOnOtherAllianceZone() || isOnShootZoneToFeed())
        && !DriverStation.getAlliance().isEmpty()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        return AllianceFlipUtil.apply(robotPose).getY()
                < FieldConstants.FieldStrategicPoses.NOT_SHOOT_LEFT_Y
            && AllianceFlipUtil.apply(robotPose).getY()
                > FieldConstants.FieldStrategicPoses.NOT_SHOOT_RIGHT_Y;
      } else {
        return AllianceFlipUtil.apply(robotPose).getY()
                < FieldConstants.FieldStrategicPoses.NOT_SHOOT_LEFT_Y
            && AllianceFlipUtil.apply(robotPose).getY()
                > FieldConstants.FieldStrategicPoses.NOT_SHOOT_RIGHT_Y;
      }
    }
    return false;
  }

  private boolean isOnShootZoneToFeed() {
    return (AllianceFlipUtil.apply(robotPose).getX()
        < FieldConstants.FieldStrategicPoses.ShootZoneX);
  }

  public void setNonOdometryMode(boolean isNonMode) {
    isNonOdometryMode = isNonMode;
  }

  public boolean isOnNonOdometryMode() {
    return isNonOdometryMode;
  }

  public void setExtendedMode(boolean isExtended) {
    Logger.recordOutput("Mode/ExtendedMode", isExtended);
    extendedMode = isExtended;
  }

  public boolean isExtendedMode() {
    return extendedMode;
  }

  public void setShootingMode(boolean isShooting) {
    Logger.recordOutput("Mode/ShootingMode", isShooting);
    shootingMode = isShooting;
  }

  public boolean isShootingMode() {
    return shootingMode;
  }

  public void setShouldShootAutonomous(boolean shouldShoot) {
    Logger.recordOutput("Mode/ShouldShoot", shouldShoot);
    shouldShootAutonomous = shouldShoot;
  }

  public boolean getShouldShootMode() {
    return shouldShootAutonomous;
  }
}
