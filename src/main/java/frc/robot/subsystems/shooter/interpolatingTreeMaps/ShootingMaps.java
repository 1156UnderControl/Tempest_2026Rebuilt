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
    shotHoodAngleMap.put(1.722, Rotation2d.fromDegrees(75));
    shotHoodAngleMap.put(2.098, Rotation2d.fromDegrees(67));
    shotHoodAngleMap.put(2.395, Rotation2d.fromDegrees(65));
    shotHoodAngleMap.put(2.7, Rotation2d.fromDegrees(63));
    shotHoodAngleMap.put(3.17, Rotation2d.fromDegrees(61));
    shotHoodAngleMap.put(3.47, Rotation2d.fromDegrees(60));
    shotHoodAngleMap.put(3.87, Rotation2d.fromDegrees(59));
    shotHoodAngleMap.put(4.15, Rotation2d.fromDegrees(58));
    shotHoodAngleMap.put(4.3, Rotation2d.fromDegrees(57));
    shotHoodAngleMap.put(4.51, Rotation2d.fromDegrees(56));
    shotHoodAngleMap.put(4.71, Rotation2d.fromDegrees(54));
    shotHoodAngleMap.put(5.0, Rotation2d.fromDegrees(53));
    shotHoodAngleMap.put(5.4, Rotation2d.fromDegrees(52));
    shotHoodAngleMap.put(5.8, Rotation2d.fromDegrees(51));
    shotHoodAngleMap.put(6.4, Rotation2d.fromDegrees(50));
    shotHoodAngleMap.put(6.8, Rotation2d.fromDegrees(49));
    shotHoodAngleMap.put(7.198, Rotation2d.fromDegrees(48));
    shotHoodAngleMap.put(7.35, Rotation2d.fromDegrees(48));

    shotFlywheelSpeedMap.put(1.23, 2400.0);
    shotFlywheelSpeedMap.put(1.722, 2500.0);
    shotFlywheelSpeedMap.put(2.098, 2600.0);
    shotFlywheelSpeedMap.put(2.395, 2600.0);
    shotFlywheelSpeedMap.put(2.7, 2800.0);
    shotFlywheelSpeedMap.put(3.17, 2850.0);
    shotFlywheelSpeedMap.put(3.47, 3000.0);
    shotFlywheelSpeedMap.put(3.87, 3100.0);
    shotFlywheelSpeedMap.put(4.15, 3100.0);
    shotFlywheelSpeedMap.put(4.3, 3100.0);
    shotFlywheelSpeedMap.put(4.51, 3100.0);
    shotFlywheelSpeedMap.put(4.71, 3150.0);
    shotFlywheelSpeedMap.put(5.0, 3250.0);
    shotFlywheelSpeedMap.put(5.4, 3400.0);
    shotFlywheelSpeedMap.put(5.8, 3550.0);
    shotFlywheelSpeedMap.put(6.4, 3700.0);
    shotFlywheelSpeedMap.put(6.8, 3850.0);
    shotFlywheelSpeedMap.put(7.198, 3950.0);
    shotFlywheelSpeedMap.put(7.35, 4050.0);

    timeOfFlightMap.put(1.25, 0.97);
    timeOfFlightMap.put(1.722, 1.1);
    timeOfFlightMap.put(2.098, 1.07);
    timeOfFlightMap.put(2.395, 1.11);
    timeOfFlightMap.put(2.7, 1.21);
    timeOfFlightMap.put(3.17, 1.19);
    timeOfFlightMap.put(3.47, 1.23);
    timeOfFlightMap.put(3.87, 1.27);
    timeOfFlightMap.put(4.15, 1.32);
    timeOfFlightMap.put(4.3, 1.27);
    timeOfFlightMap.put(4.51, 1.28);
    timeOfFlightMap.put(4.71, 1.25);
    timeOfFlightMap.put(5.0, 1.26);
    timeOfFlightMap.put(5.4, 1.33);
    timeOfFlightMap.put(5.8, 1.42);
    timeOfFlightMap.put(6.4, 1.35);
    timeOfFlightMap.put(6.8, 1.42);
    timeOfFlightMap.put(7.198, 1.42);
    timeOfFlightMap.put(7.35, 1.50);
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
