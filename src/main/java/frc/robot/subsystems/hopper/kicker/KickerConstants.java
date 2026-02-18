package frc.robot.subsystems.hopper.kicker;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class KickerConstants {
  public static final AngularVelocity VELOCITY_ERROR_ALOWED = RPM.of(100);
  public static final Distance WHEEL_DIAMETER = Inches.of(2.25);
  public static final Mass WHEEL_MASS = Kilograms.of(1.5);

  public static class setpoints {
    public static final AngularVelocity SHOOTING = RPM.of(2000);
    public static final AngularVelocity STOPPED = RPM.of(0);
    public static final AngularVelocity EXPEL = RPM.of(-1000);
  }
}
