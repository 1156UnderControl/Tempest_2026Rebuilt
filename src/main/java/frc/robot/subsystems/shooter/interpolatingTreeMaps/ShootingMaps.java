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
    shotHoodAngleMap.put(1.25, Rotation2d.fromDegrees(75));
    shotHoodAngleMap.put(1.48, Rotation2d.fromDegrees(75));
    shotHoodAngleMap.put(2.00, Rotation2d.fromDegrees(73));
    shotHoodAngleMap.put(2.55, Rotation2d.fromDegrees(70));
    shotHoodAngleMap.put(3.05, Rotation2d.fromDegrees(68));
    shotHoodAngleMap.put(3.55, Rotation2d.fromDegrees(65));
    shotHoodAngleMap.put(3.96, Rotation2d.fromDegrees(62));
    shotHoodAngleMap.put(5.54, Rotation2d.fromDegrees(52));

    // TODO: Get our own distances to RPS
    shotFlywheelSpeedMap.put(1.25, 2500.0);
    shotFlywheelSpeedMap.put(1.48, 2600.0);
    shotFlywheelSpeedMap.put(2.00, 2700.0);
    shotFlywheelSpeedMap.put(2.55, 2800.0);
    shotFlywheelSpeedMap.put(3.05, 2900.0);
    shotFlywheelSpeedMap.put(3.55, 3000.0);
    shotFlywheelSpeedMap.put(3.96, 3050.0);
    shotFlywheelSpeedMap.put(5.54, 3400.0);

    // TODO: Get our own distances TOF
    timeOfFlightMap.put(1.25, 1.0);
    timeOfFlightMap.put(1.48, 1.09);
    timeOfFlightMap.put(2.00, 1.07);
    timeOfFlightMap.put(2.55, 1.13);
    timeOfFlightMap.put(3.05, 1.21);
    timeOfFlightMap.put(3.55, 1.24);
    timeOfFlightMap.put(3.96, 1.2);
    timeOfFlightMap.put(5.54, 1.16);
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
