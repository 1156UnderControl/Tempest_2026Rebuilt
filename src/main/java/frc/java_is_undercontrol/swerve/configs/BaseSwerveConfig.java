package frc.java_is_undercontrol.swerve.configs;

import frc.java_is_undercontrol.control.ProfiledPIDConfig;

public class BaseSwerveConfig {

  public final double maxRotationRate; // Use fractions, example: 0.75 = 3/4 of a rotation

  public final SwervePathPlannerConfig pathPlannerConfig;

  public final ProfiledPIDConfig headingPidConfig;

  public final SwerveDeadbandsConfig deadbandsConfig = new SwerveDeadbandsConfig(0.05, 0.5);

  public double velocityReductionFactor = 0.4;

  public BaseSwerveConfig(
      double maxRotationSpeed,
      SwerveDeadbandsConfig deadbandsConfig,
      SwervePathPlannerConfig pathPlannerConfig,
      ProfiledPIDConfig pidHeading) {
    this.headingPidConfig = pidHeading;
    this.maxRotationRate = maxRotationSpeed;
    this.pathPlannerConfig = pathPlannerConfig;
    this.deadbandsConfig.translationDeadband = deadbandsConfig.translationDeadband;
  }

  public BaseSwerveConfig(
      double maxRotationSpeed,
      double velocityReductionFactor,
      SwerveDeadbandsConfig deadbandsConfig,
      SwervePathPlannerConfig pathPlannerConfig,
      ProfiledPIDConfig pidHeading) {
    this.headingPidConfig = pidHeading;
    this.maxRotationRate = maxRotationSpeed;
    this.velocityReductionFactor = velocityReductionFactor;
    this.pathPlannerConfig = pathPlannerConfig;
    this.deadbandsConfig.translationDeadband = deadbandsConfig.translationDeadband;
  }
}
