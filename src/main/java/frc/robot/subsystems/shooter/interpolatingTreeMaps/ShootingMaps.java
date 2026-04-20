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

    shotHoodAngleMap.put(1.3, Rotation2d.fromDegrees(75));
    shotHoodAngleMap.put(1.7, Rotation2d.fromDegrees(68));
    shotHoodAngleMap.put(2.1, Rotation2d.fromDegrees(65));
    shotHoodAngleMap.put(2.4, Rotation2d.fromDegrees(65));
    shotHoodAngleMap.put(2.7, Rotation2d.fromDegrees(64));
    shotHoodAngleMap.put(3.0, Rotation2d.fromDegrees(63));
    shotHoodAngleMap.put(3.2, Rotation2d.fromDegrees(62));
    shotHoodAngleMap.put(3.5, Rotation2d.fromDegrees(59));
    shotHoodAngleMap.put(3.9, Rotation2d.fromDegrees(56));
    shotHoodAngleMap.put(4.2, Rotation2d.fromDegrees(56));
    shotHoodAngleMap.put(4.5, Rotation2d.fromDegrees(54));
    shotHoodAngleMap.put(4.8, Rotation2d.fromDegrees(52));
    shotHoodAngleMap.put(5.1, Rotation2d.fromDegrees(51));
    shotHoodAngleMap.put(5.4, Rotation2d.fromDegrees(51));
    shotHoodAngleMap.put(5.7, Rotation2d.fromDegrees(52));
    shotHoodAngleMap.put(6.0, Rotation2d.fromDegrees(52));
    shotHoodAngleMap.put(6.3, Rotation2d.fromDegrees(53));
    shotHoodAngleMap.put(6.6, Rotation2d.fromDegrees(53));
    shotHoodAngleMap.put(6.9, Rotation2d.fromDegrees(53));
    shotHoodAngleMap.put(7.2, Rotation2d.fromDegrees(53));
    shotHoodAngleMap.put(7.5, Rotation2d.fromDegrees(53));
    //shotHoodAngleMap.put(7.8, Rotation2d.fromDegrees(53));

    shotFlywheelSpeedMap.put(1.3, 1850.0);
    shotFlywheelSpeedMap.put(1.7, 1900.0);
    shotFlywheelSpeedMap.put(2.1, 1950.0);
    shotFlywheelSpeedMap.put(2.4, 2050.0);
    shotFlywheelSpeedMap.put(2.7, 2150.0);
    shotFlywheelSpeedMap.put(3.0, 2175.0);
    shotFlywheelSpeedMap.put(3.2, 2225.0);
    shotFlywheelSpeedMap.put(3.5, 2225.0);
    shotFlywheelSpeedMap.put(3.9, 2275.0);
    shotFlywheelSpeedMap.put(4.2, 2375.0);
    shotFlywheelSpeedMap.put(4.5, 2400.0);
    shotFlywheelSpeedMap.put(4.8, 2450.0);
    shotFlywheelSpeedMap.put(5.1, 2500.0);
    shotFlywheelSpeedMap.put(5.4, 2650.0);
    shotFlywheelSpeedMap.put(5.7, 2700.0);
    shotFlywheelSpeedMap.put(6.0, 2800.0);
    shotFlywheelSpeedMap.put(6.3, 2850.0);
    shotFlywheelSpeedMap.put(6.6, 2900.0);
    shotFlywheelSpeedMap.put(6.9, 2950.0);
    shotFlywheelSpeedMap.put(7.2, 3010.0);
    shotFlywheelSpeedMap.put(7.5, 3075.0);
  //  shotFlywheelSpeedMap.put(7.8, 2950.0);

    timeOfFlightMap.put(1.3, 1.03);
    timeOfFlightMap.put(1.7, 1.08);
    timeOfFlightMap.put(2.1, 1.1);
    timeOfFlightMap.put(2.4, 1.12);
    timeOfFlightMap.put(2.7, 1.18);
    timeOfFlightMap.put(3.0, 1.20);
    timeOfFlightMap.put(3.2, 1.22);
    timeOfFlightMap.put(3.5, 1.18);
    timeOfFlightMap.put(3.9, 1.20);
    timeOfFlightMap.put(4.2, 1.25);
    timeOfFlightMap.put(4.5, 1.22);
    timeOfFlightMap.put(4.8, 1.26);
    timeOfFlightMap.put(5.1, 1.23);
    timeOfFlightMap.put(5.4, 1.30);
    timeOfFlightMap.put(5.7, 1.35);
    timeOfFlightMap.put(6.0, 1.37);
    timeOfFlightMap.put(6.3, 1.47);
    timeOfFlightMap.put(6.6, 1.48);
    timeOfFlightMap.put(6.9, 1.47);
    timeOfFlightMap.put(7.2, 1.52);
    timeOfFlightMap.put(7.5, 1.45);
  //  timeOfFlightMap.put(7.8, 1.47);

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
