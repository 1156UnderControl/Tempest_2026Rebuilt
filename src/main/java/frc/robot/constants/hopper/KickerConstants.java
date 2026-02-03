package frc.robot.constants.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
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
  public static final int MOTOR_ID = 6;

  public static final MechanismPositionConfig robotToMechanism =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(1.5))
          .withMaxRobotLength(Meters.of(0.75))
          .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)));

  public static final AngularVelocity DEADBAND_IS_AT = RPM.of(100);

  public static final class Motor {
    public static final double GEAR_RATIO = 2.5;
    public static final AngularVelocity CLOSED_LOOP_MAX_VELOCITY = RPM.of(3000);
    public static final AngularAcceleration CLOSED_LOOP_MAX_ACCELERATION =
        RotationsPerSecondPerSecond.of(150);
    public static final AngularVelocity SIM_CLOSED_LOOP_MAX_VELOCITY = DegreesPerSecond.of(90);
    public static final AngularAcceleration SIM_CLOSED_LOOP_MAX_ACCELERATION =
        DegreesPerSecondPerSecond.of(45);
    public static final Distance MECHANISM_CIRCUMFERENCE = Inches.of(2);
    public static final boolean INVERTED = true;
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(50);
  }

  public static final class Mechanism {
    public static final Distance WHEEL_DIAMETER = Inches.of(4);
    public static final Mass WHEEL_MASS = Kilograms.of(0.45);
  }

  public static final class Telemetry {
    public static final String MOTOR = "KickerMotor";
    public static final String MECHANISM = "KickerMechanism";
  }

  public static class setpoints {
    public static final AngularVelocity SHOOTING = RPM.of(3000);
    public static final AngularVelocity STOPPED = RPM.of(0);
  }
}
