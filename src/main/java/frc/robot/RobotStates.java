package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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


  public boolean isOnBlueDepotSide(){
    return (robotPose.getY() > FieldConstants.fieldWidth/2);
  }

  public boolean isOnAllianceZone(){
    return (AllianceFlipUtil.apply(robotPose).getX() < FieldConstants.FieldStrategicPoses.AllianceZoneX);
  }
}
