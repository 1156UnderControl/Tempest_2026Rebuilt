package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.java_is_undercontrol.util.AllianceFlipUtil;
import frc.robot.constants.FieldConstants;

public class RobotStates {

  private static Pose2d robotPose = Pose2d.kZero;
  private static ChassisSpeeds robotVelocity = new ChassisSpeeds();

  private static RobotStates instance;

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

  public Trigger isOnAllianceZone() {
    return new Trigger(
        () ->
            AllianceFlipUtil.apply(robotPose).getX()
                < FieldConstants.FieldStrategicPoses.AllianceZoneX);
  }

  public Trigger isOnOtherAllianceZone() {
    return new Trigger(
        () ->
            AllianceFlipUtil.apply(robotPose).getX()
                > FieldConstants.FieldStrategicPoses.OtherAllianceZoneX);
  }

  public Trigger isOnNotShootZone() {
    if (!isOnAllianceZone().getAsBoolean()
        && (isOnOtherAllianceZone().getAsBoolean() || isOnShootZoneToFeed())
        && !DriverStation.getAlliance().isEmpty()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        return new Trigger(
            () ->
                AllianceFlipUtil.apply(robotPose).getY()
                        < FieldConstants.FieldStrategicPoses.NOT_SHOOT_LEFT_Y
                    && AllianceFlipUtil.apply(robotPose).getY()
                        > FieldConstants.FieldStrategicPoses.NOT_SHOOT_RIGHT_Y);
      } else {
        return new Trigger(
            () ->
                AllianceFlipUtil.apply(robotPose).getY()
                        < FieldConstants.FieldStrategicPoses.NOT_SHOOT_LEFT_Y
                    && AllianceFlipUtil.apply(robotPose).getY()
                        > FieldConstants.FieldStrategicPoses.NOT_SHOOT_RIGHT_Y);
      }
    }
    return new Trigger(() -> false);
  }

  private boolean isOnShootZoneToFeed() {
    return (AllianceFlipUtil.apply(robotPose).getX()
        < FieldConstants.FieldStrategicPoses.ShootZoneX);
  }
}
