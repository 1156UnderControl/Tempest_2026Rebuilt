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

    // TODO: Get our own distances to HoodAngle
    shotHoodAngleMap.put(3.74, Rotation2d.fromDegrees(40.0));
    shotHoodAngleMap.put(5.74, Rotation2d.fromDegrees(40.0));
    shotHoodAngleMap.put(7.12, Rotation2d.fromDegrees(40.0));
    shotHoodAngleMap.put(8.5, Rotation2d.fromDegrees(40.0));
    shotHoodAngleMap.put(11.21, Rotation2d.fromDegrees(40.0));
    shotHoodAngleMap.put(13.0, Rotation2d.fromDegrees(40.0));

    // TODO: Get our own distances to RPS
    shotFlywheelSpeedMap.put(3.74, 2200.0);
    shotFlywheelSpeedMap.put(5.74, 2900.0);
    shotFlywheelSpeedMap.put(7.12, 3200.0);
    shotFlywheelSpeedMap.put(8.5, 3500.0);
    shotFlywheelSpeedMap.put(11.21, 4800.0);
    shotFlywheelSpeedMap.put(13.0, 5300.0);

    // TODO: Get our own distances TOF
    timeOfFlightMap.put(3.74, 1.04);
    timeOfFlightMap.put(5.74, 1.15);
    timeOfFlightMap.put(7.12, 1.25);
    timeOfFlightMap.put(8.5, 1.32);
    timeOfFlightMap.put(11.21, 1.48);
    timeOfFlightMap.put(13.0, 1.53);
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
