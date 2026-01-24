package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;

public class RobotData {

  private Supplier<Pose2d> robotPose;
  private Supplier<ChassisSpeeds> currentVelocity;

  public RobotData(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> currentSpeeds) {
    this.robotPose = robotPose;
    this.currentVelocity = currentSpeeds;
  }

  public Supplier<Pose2d> getRobotPose() {
    return robotPose;
  }

  public Supplier<ChassisSpeeds> getRobotVelocity() {
    return currentVelocity;
  }
}
