package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;

public class RobotData {

  private static Pose2d robotPose = Pose2d.kZero;
  private static ChassisSpeeds robotVelocity = new ChassisSpeeds();

  public RobotData(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> currentSpeeds) {}

  public Rotation2d getRobotRotation() {
    return robotPose.getRotation();
  }

  public Pose2d getRobotPose() {
    return robotPose;
  }

  public ChassisSpeeds getRobotVelocity() {
    return robotVelocity;
  }
}
