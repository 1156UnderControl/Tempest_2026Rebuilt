package frc.robot.subsystems.shooter.interpolatingTreeMaps;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShootingMaps implements InterpolatingTreeMaps {
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
    shotHoodAngleMap.put(1.3, Rotation2d.fromDegrees(75));
    shotHoodAngleMap.put(1.722, Rotation2d.fromDegrees(75));
    shotHoodAngleMap.put(2.098, Rotation2d.fromDegrees(67));
    shotHoodAngleMap.put(2.395, Rotation2d.fromDegrees(65));
    shotHoodAngleMap.put(2.7, Rotation2d.fromDegrees(63));
    shotHoodAngleMap.put(3.17, Rotation2d.fromDegrees(61));
    shotHoodAngleMap.put(3.47, Rotation2d.fromDegrees(60));
    shotHoodAngleMap.put(3.87, Rotation2d.fromDegrees(59));
    shotHoodAngleMap.put(4.11, Rotation2d.fromDegrees(59));
    shotHoodAngleMap.put(4.44, Rotation2d.fromDegrees(59));
    shotHoodAngleMap.put(5.05, Rotation2d.fromDegrees(59));

    // TODO: Get our own distances to RPS
    shotFlywheelSpeedMap.put(1.23, 2400.0);
    shotFlywheelSpeedMap.put(1.722, 2500.0);
    shotFlywheelSpeedMap.put(2.098, 2600.0);
    shotFlywheelSpeedMap.put(2.395, 2600.0);
    shotFlywheelSpeedMap.put(2.7, 2800.0);
    shotFlywheelSpeedMap.put(3.17, 2850.0);
    shotFlywheelSpeedMap.put(3.47, 3000.0);
    shotFlywheelSpeedMap.put(3.87, 3100.0);
    shotFlywheelSpeedMap.put(4.11, 3150.0);
    shotFlywheelSpeedMap.put(4.44, 3200.0);
    shotFlywheelSpeedMap.put(5.05, 3300.0);

    // TODO: Get our own distances TOF
    timeOfFlightMap.put(1.25, 0.97);
    timeOfFlightMap.put(1.722, 1.1);
    timeOfFlightMap.put(2.098, 1.07);
    timeOfFlightMap.put(2.395, 1.11);
    timeOfFlightMap.put(2.7, 1.21);
    timeOfFlightMap.put(3.17, 1.19);
    timeOfFlightMap.put(3.47, 1.23);
    timeOfFlightMap.put(3.87, 1.27);
    timeOfFlightMap.put(4.11, 1.29);
    timeOfFlightMap.put(4.44, 1.32);
    timeOfFlightMap.put(5.05, 1.34);
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
