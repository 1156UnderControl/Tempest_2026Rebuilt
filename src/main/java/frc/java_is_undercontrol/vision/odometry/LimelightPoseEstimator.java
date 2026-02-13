package frc.java_is_undercontrol.vision.odometry;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.java_is_undercontrol.swerve.base.odometry.OdometryEnabledSwerveSubsystem;
import frc.java_is_undercontrol.vision.Cameras.LimelightHelpers;
import frc.java_is_undercontrol.vision.Cameras.LimelightHelpers.PoseEstimate;
import java.util.Optional;

public class LimelightPoseEstimator implements PoseEstimator {
  NetworkTableInstance inst = NetworkTableInstance.getDefault();

  double fieldLength = Units.inchesToMeters(690.876);

  double fieldWidth = Units.inchesToMeters(317);

  double limitAngVelForUpdating;

  String limelightName;

  boolean only2TagsMeasurements = false;

  boolean useMegaTag1;

  private Boolean useVisionHeadingCorrection = false;

  public LimelightPoseEstimator(
      String limelightName,
      boolean only2TagsMeasurements,
      boolean useMegaTag1,
      double limitAngVelForUpdating) {
    this.limelightName = limelightName;
    this.only2TagsMeasurements = only2TagsMeasurements;
    this.useMegaTag1 = useMegaTag1;
    this.limitAngVelForUpdating = limitAngVelForUpdating;
  }

  public LimelightPoseEstimator(String limelightName, boolean only2TagsMeasurements) {
    this(limelightName, only2TagsMeasurements, false, 3);
  }

  public LimelightPoseEstimator(String limelightName) {
    this(limelightName, false);
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    try {
      if (!LimelightHelpers.getTV(this.limelightName)
          || Math.abs(OdometryEnabledSwerveSubsystem.robotAngularVelocity)
              >= limitAngVelForUpdating) {
        return Optional.empty();
      }
      PoseEstimate limelightPoseEstimate;
      if (useMegaTag1) {
        limelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.limelightName);
      } else {
        limelightPoseEstimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.limelightName);
      }
      if (limelightPoseEstimate == null) {
        return Optional.empty();
      }

      if (limelightPoseEstimate.tagCount == 1) {
        useVisionHeadingCorrection = false;
      } else {
        useVisionHeadingCorrection = false;
      }

      PoseEstimation poseEstimation = convertPoseEstimate(limelightPoseEstimate);
      if (poseOutOfField(poseEstimation)) {
        return Optional.empty();
      }
      if (only2TagsMeasurements && poseEstimation.numberOfTargetsUsed < 2) {
        return Optional.empty();
      }

      return Optional.of(poseEstimation);
    } catch (Exception e) {
      return Optional.empty();
    }
  }

  private boolean poseOutOfField(PoseEstimation pose2D) {
    double x = pose2D.estimatedPose.getX();
    double y = pose2D.estimatedPose.getY();
    return (x <= 0 || x >= fieldLength) || (y <= 0 || y >= fieldWidth);
  }

  private PoseEstimation convertPoseEstimate(PoseEstimate limelightPoseEstimate) {
    double stdDevXY =
        0.14 * Math.pow(limelightPoseEstimate.avgTagDist, 2) / limelightPoseEstimate.tagCount;
    double stdDevTheta =
        useVisionHeadingCorrection
            ? 0.14 * Math.pow(limelightPoseEstimate.avgTagDist, 2) / limelightPoseEstimate.tagCount
            : Double.POSITIVE_INFINITY;
    return new PoseEstimation(
        new Pose3d(limelightPoseEstimate.pose),
        limelightPoseEstimate.timestampSeconds,
        limelightPoseEstimate.tagCount,
        limelightPoseEstimate.avgTagDist,
        VecBuilder.fill(stdDevXY, stdDevXY, stdDevTheta));
  }

  public String getEstimatorName() {
    return this.limelightName;
  }
}
