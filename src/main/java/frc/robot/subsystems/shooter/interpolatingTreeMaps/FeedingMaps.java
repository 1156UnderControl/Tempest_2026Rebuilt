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

    shotHoodAngleMap.put(4.71, Rotation2d.fromDegrees(45.0));
    shotHoodAngleMap.put(5.35, Rotation2d.fromDegrees(45.0));
    shotHoodAngleMap.put(6.3, Rotation2d.fromDegrees(45.0));
    shotHoodAngleMap.put(7.0, Rotation2d.fromDegrees(45.0));
    shotHoodAngleMap.put(8.0, Rotation2d.fromDegrees(45.0));
    shotHoodAngleMap.put(8.7, Rotation2d.fromDegrees(45.0));
    shotHoodAngleMap.put(9.2, Rotation2d.fromDegrees(45.0));
    shotHoodAngleMap.put(10.0, Rotation2d.fromDegrees(45.0));
    shotHoodAngleMap.put(11.0, Rotation2d.fromDegrees(45.0));
    shotHoodAngleMap.put(12.0, Rotation2d.fromDegrees(45.0));
    shotHoodAngleMap.put(13.0, Rotation2d.fromDegrees(45.0));

    double rpmCorrection = 50.0;
    shotFlywheelSpeedMap.put(4.71, 2000.00 + rpmCorrection);
    shotFlywheelSpeedMap.put(5.35, 2200.00 + rpmCorrection);
    shotFlywheelSpeedMap.put(6.3, 2400.00 + rpmCorrection);
    shotFlywheelSpeedMap.put(7.0, 2600.0 + rpmCorrection);
    shotFlywheelSpeedMap.put(8.0, 2800.0 + rpmCorrection);
    shotFlywheelSpeedMap.put(8.7, 3000.0 + rpmCorrection);
    shotFlywheelSpeedMap.put(9.2, 3300.0 + rpmCorrection);
    shotFlywheelSpeedMap.put(10.0, 4000.0 + rpmCorrection);
    shotFlywheelSpeedMap.put(11.0, 4200.0 + rpmCorrection);
    // simulated
    shotFlywheelSpeedMap.put(12.0, 4400.0 + rpmCorrection);
    shotFlywheelSpeedMap.put(13.0, 4600.0 + rpmCorrection);

    timeOfFlightMap.put(4.71, 1.1);
    timeOfFlightMap.put(5.35, 1.18);
    timeOfFlightMap.put(6.3, 1.28);
    timeOfFlightMap.put(7.0, 1.32);
    timeOfFlightMap.put(8.0, 1.45);
    timeOfFlightMap.put(8.7, 1.5);
    timeOfFlightMap.put(9.2, 1.55);
    timeOfFlightMap.put(10.0, 1.58);
    timeOfFlightMap.put(11.0, 1.61);
    // simulatedq
    timeOfFlightMap.put(12.0, 1.63);
    timeOfFlightMap.put(13.0, 1.65);
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
