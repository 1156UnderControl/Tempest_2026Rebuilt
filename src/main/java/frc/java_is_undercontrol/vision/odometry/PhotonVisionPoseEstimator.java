package frc.java_is_undercontrol.vision.odometry;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.VisionConstants;
import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PhotonVisionPoseEstimator implements PoseEstimator {

  PhotonPoseEstimator photonPoseEstimator;

  PhotonCamera camera;

  boolean only2TagsMeasurements = false;

  private AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private Boolean useVisionHeadingCorrection = false;

  public PhotonVisionPoseEstimator(PhotonCamera camera, Transform3d cameraPosition) {
    this(camera, cameraPosition, false);
  }

  public PhotonVisionPoseEstimator(
      PhotonCamera camera, Transform3d cameraPosition, boolean only2TagsMeasurements) {
    this.only2TagsMeasurements = only2TagsMeasurements;
    this.camera = camera;
    this.aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, cameraPosition);
    Logger.recordOutput("PoseEstimator/PhotonVision/" + camera.getName(), new Pose3d());
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    try {
      for (var result : camera.getAllUnreadResults()) {
        Optional<EstimatedRobotPose> photonPoseEstimation =
            photonPoseEstimator.estimateCoprocMultiTagPose(result);
        if (photonPoseEstimation.isEmpty()) {
          photonPoseEstimation = photonPoseEstimator.estimateLowestAmbiguityPose(result);
          if (photonPoseEstimation.isEmpty()) {
            Logger.recordOutput("PoseEstimator/PhotonVision/" + camera.getName(), new Pose3d());
            return Optional.empty();
          }
        }

        PoseEstimation poseEstimation = convertPhotonPoseEstimation(photonPoseEstimation.get());
        if (only2TagsMeasurements && poseEstimation.numberOfTargetsUsed < 2) {
          Logger.recordOutput("PoseEstimator/PhotonVision/" + camera.getName(), new Pose3d());
          return Optional.empty();
        }
        if (poseEstimation.numberOfTargetsUsed == 1) {
          if (photonPoseEstimation.get().targetsUsed.get(0).getPoseAmbiguity() > 0.35) {
            Logger.recordOutput("PoseEstimator/PhotonVision/" + camera.getName(), new Pose3d());
            return Optional.empty();
          }

          if (poseEstimation.distanceToTag > 1) {
            Logger.recordOutput("PoseEstimator/PhotonVision/" + camera.getName(), new Pose3d());
            return Optional.empty();
          }
        }
        Logger.recordOutput(
            "PoseEstimator/PhotonVision/" + camera.getName(), poseEstimation.estimatedPose);
        return Optional.of(poseEstimation);
      }
      Logger.recordOutput("PoseEstimator/PhotonVision/" + camera.getName(), new Pose3d());
      return Optional.empty();
    } catch (Exception e) {
      Logger.recordOutput("PoseEstimator/PhotonVision/" + camera.getName(), new Pose3d());
      return Optional.empty();
    }
  }

  private PoseEstimation convertPhotonPoseEstimation(EstimatedRobotPose photonPoseEstimation) {
    double avgTagDist =
        photonPoseEstimation
            .estimatedPose
            .getTranslation()
            .getDistance(
                this.aprilTagFieldLayout
                    .getTagPose(
                        photonPoseEstimation
                            .targetsUsed
                            .get(photonPoseEstimation.targetsUsed.size() - 1)
                            .getFiducialId())
                    .get()
                    .getTranslation());

    double stdDevXY =
        VisionConstants.xyStdDevCoefficient
            * Math.pow(avgTagDist, 2)
            / photonPoseEstimation.targetsUsed.size();
    double stdDevTheta =
        useVisionHeadingCorrection
            ? VisionConstants.thetaStdDevCoefficient
                * Math.pow(avgTagDist, 2)
                / photonPoseEstimation.targetsUsed.size()
            : Double.POSITIVE_INFINITY;

    ArrayList<Integer> tagsInSight = new ArrayList<>();
    for (var targets : photonPoseEstimation.targetsUsed) {
      tagsInSight.add(targets.fiducialId);
    }
    return new PoseEstimation(
        photonPoseEstimation.estimatedPose,
        photonPoseEstimation.timestampSeconds,
        photonPoseEstimation.targetsUsed.size(),
        avgTagDist,
        VecBuilder.fill(stdDevXY, stdDevXY, stdDevTheta),
        tagsInSight);
  }

  public String getEstimatorName() {
    return this.camera.getName();
  }
}
