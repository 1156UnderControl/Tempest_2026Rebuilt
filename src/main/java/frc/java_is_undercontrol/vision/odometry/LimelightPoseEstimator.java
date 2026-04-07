package frc.java_is_undercontrol.vision.odometry;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.java_is_undercontrol.swerve.base.odometry.OdometryEnabledSwerveSubsystem;
import frc.java_is_undercontrol.vision.Cameras.LimelightHelpers;
import frc.java_is_undercontrol.vision.Cameras.LimelightHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class LimelightPoseEstimator implements PoseEstimator {
  NetworkTableInstance inst = NetworkTableInstance.getDefault();

  int[] apriltagsIDs = new int[32];

  double fieldLength = Units.inchesToMeters(690.876);

  double fieldWidth = Units.inchesToMeters(317);

  double limitAngVelForUpdating;

  String limelightName;

  boolean only2TagsMeasurements = false;

  boolean useMegaTag1;

  private Boolean useVisionHeadingCorrection = false;

  String state = "";

  private int[] apriltagsToSee = new int[0];

  String stateLoggerKey;

  String poseLoggerKey;

  boolean isRunningHdr = false;

  private int[] hdrPipelines = new int[0];

  double hdrTimePerPipeline = 1.0;

  int indexOfCurrentItemInHdrPipelines = 0;

  Timer hdrTimer = new Timer();

  String hdrEnabledLoggerKey;

  String requestedPipelineLoggerKey;

  public LimelightPoseEstimator(
      String limelightName,
      boolean only2TagsMeasurements,
      boolean useMegaTag1,
      double limitAngVelForUpdating) {
    this.limelightName = limelightName;
    this.only2TagsMeasurements = only2TagsMeasurements;
    this.useMegaTag1 = useMegaTag1;
    this.limitAngVelForUpdating = limitAngVelForUpdating;
    for (int i = 0; i < this.apriltagsIDs.length; i++) {
      this.apriltagsIDs[i] = i + 1;
    }
    this.apriltagsToSee = this.apriltagsIDs;
    this.stateLoggerKey = "PoseEstimator/Limelight/" + limelightName + "/State";
    this.poseLoggerKey = "PoseEstimator/Limelight/" + limelightName + "/Pose";
    Logger.recordOutput(this.poseLoggerKey, new Pose3d());
    this.hdrEnabledLoggerKey = "PoseEstimator/Limelight/" + limelightName + "/HdrEnabled";
    this.requestedPipelineLoggerKey = "PoseEstimator/Limelight/" + limelightName + "/RequestedPipelineId";
    Logger.recordOutput(this.hdrEnabledLoggerKey, false);
    Logger.recordOutput(this.requestedPipelineLoggerKey, 0);
  }

  public LimelightPoseEstimator(String limelightName, boolean only2TagsMeasurements) {
    this(limelightName, only2TagsMeasurements, false, 3);
  }

  public LimelightPoseEstimator(String limelightName) {
    this(limelightName, false);
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, apriltagsToSee);
    try {
      if (!LimelightHelpers.getTV(this.limelightName)) {
        this.runHdr();
        Logger.recordOutput(this.poseLoggerKey, new Pose3d());
        this.state = "NO_TARGETS";
        Logger.recordOutput(this.stateLoggerKey, this.state);
        return Optional.empty();
      }
      if (Math.abs(OdometryEnabledSwerveSubsystem.robotAngularVelocity) >= limitAngVelForUpdating) {
        this.state = "ROBOT_ANG_VEL_TOO_FAST";
        Logger.recordOutput(this.stateLoggerKey, this.state);
      }
      PoseEstimate limelightPoseEstimate;
      if (useMegaTag1) {
        limelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.limelightName);
      } else {
        limelightPoseEstimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.limelightName);
      }
      this.runHdr();
      if (limelightPoseEstimate == null) {
        Logger.recordOutput(this.poseLoggerKey, new Pose3d());
        return Optional.empty();
      }

      if (limelightPoseEstimate.tagCount == 1) {
        useVisionHeadingCorrection = false;
      } else {
        useVisionHeadingCorrection = false;
      }

      PoseEstimation poseEstimation = convertPoseEstimate(limelightPoseEstimate);
      if (poseOutOfField(poseEstimation)) {
        Logger.recordOutput(this.poseLoggerKey, new Pose3d());
        this.state = "ESTIMATION_OUT_OF_FIELD";
        Logger.recordOutput(this.stateLoggerKey, this.state);
        return Optional.empty();
      }
      if (only2TagsMeasurements && poseEstimation.numberOfTargetsUsed < 2) {
        Logger.recordOutput(this.poseLoggerKey, new Pose3d());
        this.state = "ONLY_ONE_TAG";
        Logger.recordOutput(this.stateLoggerKey, this.state);
        return Optional.empty();
      }
      Logger.recordOutput(this.poseLoggerKey, poseEstimation.estimatedPose);
      this.state = "SENDING_ESTIMATION";
      Logger.recordOutput(this.stateLoggerKey, this.state);
      return Optional.of(poseEstimation);
    } catch (Exception e) {
      Logger.recordOutput(this.poseLoggerKey, new Pose3d());
      this.state = "ERROR";
      Logger.recordOutput(this.stateLoggerKey, this.state);
      return Optional.empty();
    }
  }

  /**
   * Every int inside the array of the parameter will be a apriltag ID to be considered.
   *
   * @param aprilTagsToSee array of the tags that you want to be seen
   */
  public void onlySeeApriltagsIDs(int[] aprilTagsToSee) {
    this.apriltagsToSee = aprilTagsToSee;
  }

  public void considerAllTags() {
    this.apriltagsToSee = apriltagsIDs;
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

    ArrayList<Integer> tagsInSight = new ArrayList<>();
    for (var targets : limelightPoseEstimate.rawFiducials) {
      tagsInSight.add(targets.id);
    }
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

  public void startHdr(int[] hdrPipelines, double hdrTimePerPipeline){
    this.hdrPipelines = hdrPipelines;
    this.hdrTimePerPipeline = hdrTimePerPipeline;
    this.indexOfCurrentItemInHdrPipelines = 0;
    this.isRunningHdr = true;
    Logger.recordOutput(this.hdrEnabledLoggerKey, true);
    LimelightHelpers.setPipelineIndex(this.limelightName, hdrPipelines[indexOfCurrentItemInHdrPipelines]);
    Logger.recordOutput(this.requestedPipelineLoggerKey, hdrPipelines[indexOfCurrentItemInHdrPipelines]);
    this.hdrTimer = new Timer();
    this.hdrTimer.reset();
    this.hdrTimer.start();
  }

  public void stopHdr(int pipelineIndexToUse){
    this.isRunningHdr = false;
    Logger.recordOutput(this.hdrEnabledLoggerKey, false);
    LimelightHelpers.setPipelineIndex(this.limelightName, pipelineIndexToUse);
    Logger.recordOutput(this.requestedPipelineLoggerKey, pipelineIndexToUse);
    this.hdrTimer.stop();
  }

  private void runHdr(){
    // HDR (High Dynamic Range) is the term used for a trick that phones normally use when the 
    // dynamic range (difference between bright and dark) is too much for the sensor. What they do
    // is take multiple pictures with different exposure levels and stitch them together to get
    // a picture that is well exposed. We are trying to do something similar here by running
    // different pipelines every 1/4 of a second, ensuring we get at least some tag in any situation
    if(!this.isRunningHdr) return;
    if(!this.hdrTimer.hasElapsed(this.hdrTimePerPipeline)) return;
    this.indexOfCurrentItemInHdrPipelines++;
    if(this.indexOfCurrentItemInHdrPipelines >= this.hdrPipelines.length){
      this.indexOfCurrentItemInHdrPipelines = 0;
    }
    LimelightHelpers.setPipelineIndex(this.limelightName, hdrPipelines[indexOfCurrentItemInHdrPipelines]);
    Logger.recordOutput(this.requestedPipelineLoggerKey, hdrPipelines[indexOfCurrentItemInHdrPipelines]);
    this.hdrTimer.reset();
  }
}
