package frc.robot.constants.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

public class FlyWheelsConstants {
  public static final DCMotor FLYWHEEL_MOTOR = DCMotor.getKrakenX44(2);
  public static final MechanismGearing MOTOR_GEARING =
      new MechanismGearing(GearBox.fromReductionStages(1));
  public static final Mass FLYWHEEL_MASS = Pounds.of(1);
  public static final Distance FLYWHEEL_SIZE = Inches.of(0.11);
  public static final Voltage MAX_VOLTS = Volts.of(12.0);

  public static class Velocity {
    public static final AngularVelocity VELOCITY_ERROR_ALLOWED = RotationsPerSecond.of(1.0);
  }

  public static class ClosedLoopController {
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;
  }
}
