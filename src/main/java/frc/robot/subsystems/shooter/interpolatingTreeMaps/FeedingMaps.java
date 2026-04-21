package frc.robot.subsystems.shooter.interpolatingTreeMaps;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class FeedingMaps implements InterpolatingTreeMaps {
  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private static double minDistance;
  private static double maxDistance;

  static {
    minDistance = 1.34;
    maxDistance = 5.60;

    shotHoodAngleMap.put(4.8, Rotation2d.fromDegrees(44.0));
    shotHoodAngleMap.put(5.4, Rotation2d.fromDegrees(44.0));
    shotHoodAngleMap.put(6.3, Rotation2d.fromDegrees(44.0));
    shotHoodAngleMap.put(7.0, Rotation2d.fromDegrees(44.0));
    shotHoodAngleMap.put(8.0, Rotation2d.fromDegrees(44.0));
    shotHoodAngleMap.put(8.7, Rotation2d.fromDegrees(44.0));
    shotHoodAngleMap.put(9.2, Rotation2d.fromDegrees(44.0));
    shotHoodAngleMap.put(10.0, Rotation2d.fromDegrees(44.0));
    shotHoodAngleMap.put(11.0, Rotation2d.fromDegrees(44.0));
    shotHoodAngleMap.put(12.0, Rotation2d.fromDegrees(47.0));
    shotHoodAngleMap.put(13.0, Rotation2d.fromDegrees(47.0));
    shotHoodAngleMap.put(14.0, Rotation2d.fromDegrees(49.0));
    shotHoodAngleMap.put(15.0, Rotation2d.fromDegrees(50.0));

    double rpmCorrection = 0.0;
    shotFlywheelSpeedMap.put(4.8, 1900.00 + rpmCorrection);

    shotFlywheelSpeedMap.put(5.4, 2000.0 + rpmCorrection);
    shotFlywheelSpeedMap.put(6.3, 2250.0 + rpmCorrection);
    shotFlywheelSpeedMap.put(7.0, 2400.0 + rpmCorrection);
    shotFlywheelSpeedMap.put(8.0, 2650.0 + rpmCorrection);
    shotFlywheelSpeedMap.put(8.7, 2925.0 + rpmCorrection);
    shotFlywheelSpeedMap.put(9.2, 3150.0 + rpmCorrection);
    shotFlywheelSpeedMap.put(10.0, 3500.0 + rpmCorrection);
    shotFlywheelSpeedMap.put(11.0, 3900.0 + rpmCorrection);
    shotFlywheelSpeedMap.put(12.0, 4600.0 + rpmCorrection);
    shotFlywheelSpeedMap.put(13.0, 5100.0 + rpmCorrection);
    shotFlywheelSpeedMap.put(14.0, 5400.0 + rpmCorrection);
    shotFlywheelSpeedMap.put(15.0, 5600.0 + rpmCorrection);

    timeOfFlightMap.put(4.8, 1.1);
    timeOfFlightMap.put(5.4, 1.18);
    timeOfFlightMap.put(6.3, 1.31);
    timeOfFlightMap.put(7.0, 1.32);
    timeOfFlightMap.put(8.0, 1.43);
    timeOfFlightMap.put(8.7, 1.52);
    timeOfFlightMap.put(9.2, 1.54);
    timeOfFlightMap.put(10.0, 1.62);
    timeOfFlightMap.put(11.0, 1.72);
    timeOfFlightMap.put(12.0, 1.86);
    timeOfFlightMap.put(13.0, 1.94);
    timeOfFlightMap.put(14.0, 2.07);
    timeOfFlightMap.put(14.0, 2.15);
  }

  public InterpolatingTreeMap<Double, Rotation2d> getHoodAngleMap() {
    return shotHoodAngleMap;
  }

  public InterpolatingDoubleTreeMap getFlywheelSpeedMap() {
    return shotFlywheelSpeedMap;
  }

  public InterpolatingDoubleTreeMap getTimeOfFlightMap() {
    return timeOfFlightMap;
  }

  public double getMinDistance() {
    return minDistance;
  }

  public double getMaxDistance() {
    return maxDistance;
  }
}
