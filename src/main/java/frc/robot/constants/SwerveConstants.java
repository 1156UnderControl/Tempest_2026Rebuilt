package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.java_is_undercontrol.control.PIDConfig;

public class SwerveConstants {
  public static final PIDConfig MOVE_TO_POSE_TRANSLATION_PID = new PIDConfig(5, 0, 0, 0, 0);
  public static final Constraints MOVE_TO_POSE_TRANSLATION_CONSTRAINTS = new Constraints(4, 3);
  public static final Constraints MOVE_TO_POSE_X_CONSTRAINTS = new Constraints(4, 3);
  public static final Constraints MOVE_TO_POSE_Y_CONSTRAINTS = new Constraints(4, 3);

  public enum PoseEstimatorState {
    GLOBAL_POSE_ESTIMATION,
    AUTO_POSE_ESTIMATION;
  }
}
