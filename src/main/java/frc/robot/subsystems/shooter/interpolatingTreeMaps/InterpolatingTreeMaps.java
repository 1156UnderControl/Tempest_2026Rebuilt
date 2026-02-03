package frc.robot.subsystems.shooter.interpolatingTreeMaps;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

public interface InterpolatingTreeMaps {

  public InterpolatingTreeMap<Double, Rotation2d> getHoodAngleMap();

  public InterpolatingDoubleTreeMap getFlywheelSpeedMap();

  public InterpolatingDoubleTreeMap getTimeOfFlightMap();

  public double getMaxDistance();

  public double getMinDistance();
}
