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

  private double distanceToRefuse1TagMeasurement;

  private String state;

  private String keyPoseEstimatedLog;

  private String keyStateLog;

  private PhotonPoseEstimatorStrategies strategy;

  private int[] filteredApriltags = new int[0];

  public PhotonVisionPoseEstimator(PhotonCamera camera, Transform3d cameraPosition) {
    this(camera, cameraPosition, false, 1, PhotonPoseEstimatorStrategies.MULTITAG_COPROCESSOR);
  }

  public PhotonVisionPoseEstimator(
      PhotonCamera camera,
      Transform3d cameraPosition,
      boolean only2TagsMeasurements,
      double distanceToRefuse1TagMeasurement,
      PhotonPoseEstimatorStrategies strategy) {
    this.only2TagsMeasurements = only2TagsMeasurements;
    this.camera = camera;
    this.aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, cameraPosition);
    this.distanceToRefuse1TagMeasurement = distanceToRefuse1TagMeasurement;
    this.state = "INITIALIZED";
    this.strategy = strategy;
    this.keyPoseEstimatedLog = "PoseEstimator/PhotonVision/" + camera.getName() + "/EstimatedPose";
    this.keyStateLog = "PoseEstimator/PhotonVision/" + camera.getName() + "/StateMeasure";
    Logger.recordOutput(this.keyPoseEstimatedLog, new Pose3d());
    Logger.recordOutput(this.keyPoseEstimatedLog, new Pose3d());
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    try {
      for (var result : camera.getAllUnreadResults()) {
        Optional<EstimatedRobotPose> photonPoseEstimation;
        switch (strategy) {
          case MULTITAG_COPROCESSOR:
            photonPoseEstimation = photonPoseEstimator.estimateCoprocMultiTagPose(result);
            break;
          case PNP_CONSTRAINED:
            photonPoseEstimator.addHeadingData(
                result.getTimestampSeconds(), referencePose.getRotation());
            photonPoseEstimation =
                photonPoseEstimator.estimateConstrainedSolvepnpPose(
                    result,
                    camera.getCameraMatrix().orElseThrow(),
                    camera.getDistCoeffs().orElseThrow(),
                    new Pose3d(referencePose),
                    false,
                    1);
            break;
          default:
            photonPoseEstimation = photonPoseEstimator.estimateCoprocMultiTagPose(result);
            break;
        }
        if (photonPoseEstimation.isEmpty() || !result.hasTargets()) {
          photonPoseEstimation = photonPoseEstimator.estimateLowestAmbiguityPose(result);
          if (photonPoseEstimation.isEmpty()) {
            state = "NO_TARGETS";
            Logger.recordOutput(keyStateLog, state);
            Logger.recordOutput(this.keyPoseEstimatedLog, new Pose3d());
            return Optional.empty();
          }
        }

        for (var targets : result.getTargets()) {
          for (int idFilter : this.filteredApriltags) {
            if (targets.fiducialId == idFilter) {
              state = "FILTER_ID_" + idFilter;
              Logger.recordOutput(keyStateLog, state);
              Logger.recordOutput(this.keyPoseEstimatedLog, new Pose3d());
              return Optional.empty();
            }
          }
        }

        PoseEstimation poseEstimation = convertPhotonPoseEstimation(photonPoseEstimation.get());
        if (only2TagsMeasurements && poseEstimation.numberOfTargetsUsed < 2) {
          state = "TWO_TAGS_ONLY";
          Logger.recordOutput(keyStateLog, state);
          Logger.recordOutput(this.keyPoseEstimatedLog, new Pose3d());
          return Optional.empty();
        }
        if (poseEstimation.numberOfTargetsUsed == 1) {
          if (photonPoseEstimation.get().targetsUsed.get(0).getPoseAmbiguity() > 0.35) {
            state = "ONE_TAG_TOO_AMBIGUOUS";
            Logger.recordOutput(keyStateLog, state);
            Logger.recordOutput(this.keyPoseEstimatedLog, new Pose3d());
            return Optional.empty();
          }

          if (poseEstimation.distanceToTag > this.distanceToRefuse1TagMeasurement) {
            state = "ONE_TAG_TOO_FAR";
            Logger.recordOutput(keyStateLog, state);
            Logger.recordOutput(this.keyPoseEstimatedLog, new Pose3d());
            return Optional.empty();
          }
        }
        state = "SENDING_ESTIMATION";
        Logger.recordOutput(keyStateLog, state);
        Logger.recordOutput(this.keyPoseEstimatedLog, poseEstimation.estimatedPose);
        return Optional.of(poseEstimation);
      }
      return Optional.empty();
    } catch (Exception e) {
      state = "EXCEPTION THROWED";
      Logger.recordOutput(keyStateLog, state);
      Logger.recordOutput(this.keyPoseEstimatedLog, new Pose3d());
      return Optional.empty();
    }
  }

  /**
   * Every int inside the array of the parameter will be a apriltag ID that will be IGNORED.
   *
   * @param aprilTagsToFilter array of the tags that you want to be IGNORED
   */
  public void ignoreApriltags(int[] aprilTagsToFilter) {
    this.filteredApriltags = aprilTagsToFilter;
  }

  public void considerAllTags() {
    this.filteredApriltags = new int[0];
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
