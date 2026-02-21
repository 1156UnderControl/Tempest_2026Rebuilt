package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  public static final String frontRightArducamName = "Arducam-Front-Right";
  public static final String frontLeftArducamName = "Arducam-Front-Left";

  public static final String backRightLimelightName = "limelight-bright";
  public static final String backLeftLimelightName = "limelight-bleft";
  public static final double xyStdDevCoefficient = 0.04;
  public static final double thetaStdDevCoefficient = 0.08;

  public static Transform3d robotToCamArducamFrontRight =
      new Transform3d(
          new Translation3d((0.2), (-0.60), 0.153222),
          new Rotation3d(0, Units.degreesToRadians(24.584), Units.degreesToRadians(301)));

  public static Transform3d robotToCamArducamFrontLeft =
      new Transform3d(
          new Translation3d((-0.307143), (-0.275344), 0.251786),
          new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(200)));
}
