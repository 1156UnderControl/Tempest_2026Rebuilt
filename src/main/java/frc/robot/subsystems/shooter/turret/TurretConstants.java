package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class TurretConstants {
  public static final int TURRET_MOTOR_ID = 7;

  public static final MotorType MOTOR_TYPE = MotorType.kBrushless;

  public static final TelemetryVerbosity TELEMETRY = TelemetryVerbosity.HIGH;

  public static final DCMotor DC_MOTOR = DCMotor.getNeoVortex(1);

  public static final Distance TURRET_SIZE = Meters.of(0.25);

  public static final Mass TURRET_MASS = Kilograms.of(6);

  public static final String TURRET_MECHANISM_NAME = "Turret";

  public static final Voltage MAX_VOLTS = Volts.of(12.0);

  public static final int TURRET_GEAR_TEETH = 96;

  public static final MechanismGearing GEARING =
      new MechanismGearing(GearBox.fromReductionStages(60.3428));

  public static final double PHASE_DELAY = 0.03;

  public static final boolean INVERTED = false;

  public static final Current STATOR_CURRENT_LIMIT = Amps.of(60);

  public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(360);

  public static final AngularAcceleration ACCELERATION = DegreesPerSecondPerSecond.of(200);

  public static final Transform2d TURRET_OFFSET_FROM_ROBOT_CENTER =
      new Transform2d(-0.127, -0.18415, Rotation2d.kZero);

  public static final Angle ANGLE_ERROR_ALLOWED = Degrees.of(1.0);

  public static final Angle TURRET_MIN_ANGLE = Degrees.of(-180);

  public static final Angle TURRET_MAX_ANGLE = Degrees.of(180);

  public static final Time CLOSED_RAMP_RATE = Seconds.of(0.5);

  public static class FeedBackController {
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static class SIM {
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
    }
  }

  public static class FeedForwardController {
    public static final double kV = 0;
    public static final double kS = 0;
    public static final double kA = 0;

    public static class SIM {
      public static final double kV = 200;
      public static final double kS = 0;
      public static final double kA = 0;
    }
  }
}
