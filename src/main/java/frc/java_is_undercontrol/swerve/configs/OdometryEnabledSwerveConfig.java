package frc.java_is_undercontrol.swerve.configs;

import frc.java_is_undercontrol.control.ProfiledPIDConfig;
import frc.java_is_undercontrol.vision.odometry.PoseEstimator;

public class OdometryEnabledSwerveConfig extends BaseSwerveConfig {

  public final PoseEstimator autonomousPoseEstimator;

  public final PoseEstimator teleoperatedPoseEstimator;

  public final MoveToPosePIDConfig moveToPosePIDConfig;

  public OdometryEnabledSwerveConfig(
      double maxRotationSpeed,
      SwerveDeadbandsConfig deadbandsConfig,
      SwervePathPlannerConfig pathPlannerConfig,
      PoseEstimator autonomousPoseEstimator,
      PoseEstimator teleoperatedPoseEstimator,
      ProfiledPIDConfig headingPIDConfig,
      MoveToPosePIDConfig moveToPosePIDConfig) {
    super(maxRotationSpeed, deadbandsConfig, pathPlannerConfig, headingPIDConfig);
    this.autonomousPoseEstimator = autonomousPoseEstimator;
    this.teleoperatedPoseEstimator = teleoperatedPoseEstimator;
    this.moveToPosePIDConfig = moveToPosePIDConfig;
  }

  public OdometryEnabledSwerveConfig(
      double maxRotationSpeed,
      double velocityReductionFactor,
      SwerveDeadbandsConfig deadbandsConfig,
      SwervePathPlannerConfig pathPlannerConfig,
      PoseEstimator autonomousPoseEstimator,
      PoseEstimator teleoperatedPoseEstimator,
      ProfiledPIDConfig headingPIDConfig,
      MoveToPosePIDConfig moveToPosePIDConfig) {
    super(
        maxRotationSpeed,
        velocityReductionFactor,
        deadbandsConfig,
        pathPlannerConfig,
        headingPIDConfig);
    this.autonomousPoseEstimator = autonomousPoseEstimator;
    this.teleoperatedPoseEstimator = teleoperatedPoseEstimator;
    this.moveToPosePIDConfig = moveToPosePIDConfig;
  }
}
