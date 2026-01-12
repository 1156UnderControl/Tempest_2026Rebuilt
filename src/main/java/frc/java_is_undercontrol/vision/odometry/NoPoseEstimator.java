package frc.java_is_undercontrol.vision.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;

public class NoPoseEstimator implements PoseEstimator {

  @Override
  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    return Optional.empty();
  }

  @Override
  public String getEstimatorName() {
    return "";
  }
}
