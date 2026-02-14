package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  public static final String frontRightArducamName = "Arducam-Front-Right";
  public static final String backRightArducamName = "Arducam-Back-Right";

  public static final String frontLeftLimelightName = "limelight-fleft";
  public static final String backLeftLimelightName = "limelight-bleft";
  public static final double xyStdDevCoefficient = 0.04;
  public static final double thetaStdDevCoefficient = 0.08;

  public static Transform3d robotToCamArducamFrontRight =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-3.260735), Units.inchesToMeters(-12.958429), 7.641844),
          new Rotation3d(
              0, Units.degreesToRadians(-14.245433), Units.degreesToRadians(25.418201 - 90)));

  public static Transform3d robotToCamArducamBackRight =
      new Transform3d(
          new Translation3d(
             (-0.275339), (-0.281729), 0.251841),
          new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(200)));
}
