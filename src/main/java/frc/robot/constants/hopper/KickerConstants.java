package frc.robot.constants.hopper;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class KickerConstants {
  public static final AngularVelocity VELOCITY_ERROR_ALOWED = RPM.of(100);

  public static class setpoints {
    public static final AngularVelocity SHOOTING = RPM.of(3000);
    public static final AngularVelocity STOPPED = RPM.of(0);
  }
}
