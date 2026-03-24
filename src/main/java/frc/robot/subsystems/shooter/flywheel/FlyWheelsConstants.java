package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

public class FlyWheelsConstants {
  public static final DCMotor FLYWHEEL_MOTOR = DCMotor.getKrakenX60Foc(2);
  public static final MechanismGearing MOTOR_GEARING =
      new MechanismGearing(GearBox.fromReductionStages(0.71));
  public static final Mass FLYWHEEL_MASS = Kilograms.of(1.8);
  public static final Distance FLYWHEEL_SIZE = Inches.of(2);
  public static final Voltage MAX_VOLTS = Volts.of(12.0);

  public static class Velocity {
    public static final AngularVelocity VELOCITY_ERROR_ALLOWED_FOR_FEED = RPM.of(1000);
    public static final AngularVelocity VELOCITY_ERROR_ALLOWED = RPM.of(300);
    public static final AngularVelocity VELOCITY_ERROR_ALLOWED_FIRST_SHOT = RPM.of(500);
  }

  public static final double FEEDFORWARD_TORQUE_CURRENT_A = 3;

  public static class ClosedLoopController {
    public static final double kP = 8;
    public static final double kI = 0;
    public static final double kD = 0;
  }
}
