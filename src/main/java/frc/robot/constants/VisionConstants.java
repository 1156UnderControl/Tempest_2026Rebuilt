package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final String leftBackArducamName = "Arducam-Left-Back";
  public static final double xyStdDevCoefficient = 0.04;
  public static final double thetaStdDevCoefficient = 0.08;

  public static Transform3d robotToCamArducamLeftBack = new Transform3d(new Translation3d(Units.inchesToMeters(-3.260735), Units.inchesToMeters(-12.958429), 7.641844),
      new Rotation3d(0, Units.degreesToRadians(-14.245433),
          Units.degreesToRadians(25.418201-90)));
}
