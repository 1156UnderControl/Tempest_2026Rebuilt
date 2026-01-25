package frc.robot.constants.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class TurretConstants {
  public static final int TURRET_MOTOR_ID = 20;
  public static final String MOTOR_NAME = "TurretMotor";
  public static final DCMotor TURRET_MOTOR = DCMotor.getNeoVortex(1);
  public static final MotorMode MOTOR_MODE = MotorMode.BRAKE;
  public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
  public static final MechanismGearing MOTOR_GEARING =
      new MechanismGearing(GearBox.fromReductionStages(360));
  public static final ControlMode CONTROL_MODE = ControlMode.CLOSED_LOOP;
  public static final TelemetryVerbosity TELEMETRY = TelemetryVerbosity.HIGH;
  public static final Current STATOR_CURRENT = Amps.of(40);
  public static final Time CLOSED_RAMP_RATE = Seconds.of(0.5);
  public static final Time OPEN_RAMP_RATE = Seconds.of(0.5);
  public static final Mass TURRET_MASS = Kilograms.of(6);
  public static final Distance TURRET_SIZE = Meters.of(0.25);

  public static class Velocity {
    public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(360);
    public static final AngularAcceleration ACCELERATION = DegreesPerSecondPerSecond.of(200);
  }

  public static class Position {
    public static final Translation2d TURRET_OFFSET_FROM_ROBOT_CENTER = new Translation2d(0, 0);
    public static final boolean MOTOR_INVERTED = false;
    public static final double ANGLE_ERROR_ALLOWED = 1.0;
    public static final Angle TURRET_MIN_ANGLE = Degrees.of(-200);
    public static final Angle TURRET_MAX_ANGLE = Degrees.of(200);
    public static final Angle STARTING_POSITION = Degrees.of(0);
  }

  public static class ClosedLoopController {
    public static final double P = 100;
    public static final double I = 0;
    public static final double D = 0;
  }

  public static class FeedForward {
    public static final double KS = 0.5;
    public static final double KV = 0.5;
    public static final double KA = 0;
  }
}
