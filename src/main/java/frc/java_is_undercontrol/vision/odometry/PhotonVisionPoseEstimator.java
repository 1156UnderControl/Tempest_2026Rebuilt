package frc.java_is_undercontrol.vision.odometry;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVisionPoseEstimator implements PoseEstimator {

  PhotonPoseEstimator photonPoseEstimator;

  PhotonCamera camera;

  boolean only2TagsMeasurements = false;

  private AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private Boolean useVisionHeadingCorrection = false;

  public PhotonVisionPoseEstimator(PhotonCamera camera, Transform3d cameraPosition) {
    this.camera = camera;
    this.aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    this.photonPoseEstimator =
        new PhotonPoseEstimator(
            this.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraPosition);
    this.photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public PhotonVisionPoseEstimator(
      PhotonCamera camera, Transform3d cameraPosition, boolean only2TagsMeasurements) {
    this.only2TagsMeasurements = only2TagsMeasurements;
    this.camera = camera;
    this.aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    this.photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.CONSTRAINED_SOLVEPNP, cameraPosition);
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    try {
      this.photonPoseEstimator.setReferencePose(referencePose);
      this.photonPoseEstimator.addHeadingData(
          Timer.getFPGATimestamp(), referencePose.getRotation());
      List<PhotonPipelineResult> results = camera.getAllUnreadResults();
      if (results.isEmpty()) {
        return Optional.empty();
      }
      Optional<EstimatedRobotPose> photonPoseEstimation =
          this.photonPoseEstimator.update(results.get(results.size() - 1));
      if (!photonPoseEstimation.isPresent()) {
        return Optional.empty();
      }

      PoseEstimation poseEstimation = convertPhotonPoseEstimation(photonPoseEstimation.get());
      if (only2TagsMeasurements && poseEstimation.numberOfTargetsUsed < 2) {
        return Optional.empty();
      }
      if (poseEstimation.numberOfTargetsUsed == 1) {
        if (photonPoseEstimation.get().targetsUsed.get(0).getPoseAmbiguity() > 0.35) {
          return Optional.empty();
        }

        if (poseEstimation.distanceToTag > 2.7) {
          return Optional.empty();
        }
      }
      return Optional.of(poseEstimation);
    } catch (Exception e) {
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
    return new PoseEstimation(
        photonPoseEstimation.estimatedPose,
        photonPoseEstimation.timestampSeconds,
        photonPoseEstimation.targetsUsed.size(),
        avgTagDist,
        VecBuilder.fill(stdDevXY, stdDevXY, stdDevTheta));
  }

  public String getEstimatorName() {
    return this.camera.getName();
  }
}
