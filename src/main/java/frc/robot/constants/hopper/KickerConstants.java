package frc.robot.constants.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import yams.mechanisms.config.MechanismPositionConfig;

public final class KickerConstants {
  public static final AngularVelocity VELOCITY_ERROR_ALOWED = RPM.of(100);
  public static final Distance WHEEL_DIAMETER = Inches.of(2.25);
  public static final Mass WHEEL_MASS = Pounds.of(0.08);

  public static class setpoints {
    public static final AngularVelocity SHOOTING = RPM.of(3000);
    public static final AngularVelocity STOPPED = RPM.of(0);
  }
}
