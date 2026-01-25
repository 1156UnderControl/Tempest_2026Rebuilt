package frc.robot.constants.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.ImmutableMomentOfInertia;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class HoodConstants {
  public static final int HOOD_MOTOR_ID = 21;
  public static final String MOTOR_NAME = "HoodMotor";
  public static final DCMotor HOOD_MOTOR = DCMotor.getNEO(1);
  public static final MotorMode MOTOR_MODE = MotorMode.BRAKE;
  public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
  public static final MechanismGearing MOTOR_GEARING =
      new MechanismGearing(GearBox.fromReductionStages(360));
  public static final ControlMode CONTROL_MODE = ControlMode.CLOSED_LOOP;
  public static final TelemetryVerbosity TELEMETRY = TelemetryVerbosity.HIGH;
  public static final Current STATOR_CURRENT = Amps.of(40);
  public static final Time CLOSED_RAMP_RATE = Seconds.of(0.5);
  public static final Time OPEN_RAMP_RATE = Seconds.of(0.5);
  public static final Mass HOOD_MASS = Kilograms.of(2.5);
  public static final Distance HOOD_SIZE = Meters.of(0.4);

  public static class Velocity {
    public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(50);
    public static final AngularAcceleration ACCELERATION = DegreesPerSecondPerSecond.of(30);
  }

  public static class Position {
    public static final boolean MOTOR_INVERTED = false;
    public static final double ANGLE_ERROR_ALLOWED = 1.0;
    public static final Angle HOOD_MIN_ANGLE = Degrees.of(15);
    public static final Angle HOOD_MAX_ANGLE = Degrees.of(50);
    public static final Angle STARTING_POSITION = Degrees.of(15);
  }

  public static class ClosedLoopController {
    public static final double P = 100;
    public static final double I = 0;
    public static final double D = 0;

    public static class SIM {
      public static final double P = 100;
      public static final double I = 0;
      public static final double D = 0;
    }
  }

  public static class FeedForward {
    public static final double KS = 0.5;
    public static final double KV = 0.5;
    public static final double KA = 0;

    public static class SIM{
      public static final double KS = 0.5;
      public static final double KV = 0.5;
      public static final double KA = 0;
    }
  }
}
