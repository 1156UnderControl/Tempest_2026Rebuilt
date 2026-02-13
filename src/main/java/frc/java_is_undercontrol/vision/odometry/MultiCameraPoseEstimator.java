package frc.java_is_undercontrol.vision.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class MultiCameraPoseEstimator implements PoseEstimator {

  List<PoseEstimator> poseEstimatorsList;

  int numberOfTargetsUsed = 0;

  String multiPoseEstimatorName;

  public MultiCameraPoseEstimator(
      List<PoseEstimator> poseEstimators, String multiPoseEstimatorName) {
    this.poseEstimatorsList = poseEstimators;
    this.multiPoseEstimatorName = multiPoseEstimatorName;
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    ArrayList<Pose2d> pose2Ds = new ArrayList<>();
    ArrayList<PoseEstimation> poseEstimations = new ArrayList<>();
    for (PoseEstimator poseEstimator : this.poseEstimatorsList) {
      Optional<PoseEstimation> possiblePoseEstimation =
          poseEstimator.getEstimatedPose(referencePose);
      if (possiblePoseEstimation.isPresent()) {
        PoseEstimation poseEstimation = possiblePoseEstimation.get();
        for (int i = 0; i < poseEstimation.numberOfTargetsUsed; i++) {
          pose2Ds.add(poseEstimation.estimatedPose.toPose2d());
          poseEstimations.add(poseEstimation);
        }
      }
    }
    if (poseEstimations.size() < 1) {
      return Optional.empty();
    }
    PoseEstimation poseEstimation;
    if (poseEstimations.size() > 1) {
      poseEstimations.sort(new ComparatorPoseEstimationTagCount());
      double TagCount1 = poseEstimations.get(poseEstimations.size() - 1).numberOfTargetsUsed;
      double TagCount2 = poseEstimations.get(poseEstimations.size() - 2).numberOfTargetsUsed;
      if (TagCount1 != TagCount2) {
        double biggerDistance =
            Math.max(
                poseEstimations.get(poseEstimations.size() - 1).distanceToTag,
                poseEstimations.get(poseEstimations.size() - 2).distanceToTag);
        if (biggerDistance == poseEstimations.get(poseEstimations.size() - 1).distanceToTag) {
          poseEstimation = poseEstimations.get(poseEstimations.size() - 2);
        } else {
          poseEstimation = poseEstimations.get(poseEstimations.size() - 1);
        }
      } else {
        poseEstimation = poseEstimations.get(poseEstimations.size() - 1);
      }
      int bestHubScore = 0;
      int bestHubScoreIndex = 0;
      for (int i = 0; i < poseEstimations.size(); i++) {
        PoseEstimation pose = poseEstimations.get(i);
        boolean isRedAlliance =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        double[] priorityTags = isRedAlliance ? new double[] {9, 10} : new double[] {25, 26};
        int hubScore = 0;
        for (int tagSeen : pose.tagsSeen) {
          if (tagSeen == priorityTags[0] || tagSeen == priorityTags[1]) {
            hubScore++;
          }
          if (bestHubScore < hubScore) {
            bestHubScore = hubScore;
            bestHubScoreIndex = i;
          }
        }
      }
      poseEstimations.add(poseEstimations.get(bestHubScoreIndex));
    } else {
      poseEstimation = poseEstimations.get(poseEstimations.size() - 1);
    }
    return Optional.of(poseEstimation);
  }

  @Override
  public String getEstimatorName() {
    return this.multiPoseEstimatorName;
  }
}
