package frc.java_is_undercontrol.swerve.configs;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

public class SwervePathPlannerConfig {
  public PIDConstants translationPid;

  public PIDConstants anglePid;

  public double maxModuleSpeed;

  public final PathConstraints pathFinderConstraints;

  public SwervePathPlannerConfig(
      PIDConstants translationPid, PIDConstants anglePid, PathConstraints pathFindeConstraints) {
    this.translationPid = translationPid;
    this.anglePid = anglePid;
    this.pathFinderConstraints = pathFindeConstraints;
  }
}
