package frc.java_is_undercontrol.vision.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;

public interface PoseEstimator {

  public String getEstimatorName();

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose);
}
