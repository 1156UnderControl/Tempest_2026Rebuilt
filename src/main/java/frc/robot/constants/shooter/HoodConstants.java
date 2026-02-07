package frc.robot.constants.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

public class HoodConstants {
  public static final DCMotor HOOD_MOTOR = DCMotor.getNEO(1);
  public static final MechanismGearing MOTOR_GEARING =
      new MechanismGearing(GearBox.fromReductionStages(11.66, 15.3743));
  public static final Mass HOOD_MASS = Kilograms.of(2.5);
  public static final Distance HOOD_SIZE = Meters.of(0.4);
  public static final Voltage MAX_VOLTS = Volts.of(12.0);

  public static class Position {
    public static final Angle HOOD_MIN_ANGLE = Degrees.of(40);
    public static final Angle HOOD_MAX_ANGLE = Degrees.of(75);
    public static final Angle ANGLE_ERROR_ALLOWED = Degrees.of(0.2);
  }

  public static class ClosedLoopController {
    public static final double kP = 290;
    public static final double kI = 3;
    public static final double kD = 0;
  }
}
