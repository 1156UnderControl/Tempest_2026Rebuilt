package frc.robot.constants;

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
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class SubsystemConstantsTemplate {
  public static final int MOTOR_ID = 0;
  public static final String MOTOR_NAME = "";
  public static final DCMotor DC_MOTOR = DCMotor.getNeoVortex(1);
  public static final MotorMode MOTOR_MODE = MotorMode.BRAKE;
  public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
  public static final MechanismGearing MOTOR_GEARING =
      new MechanismGearing(GearBox.fromReductionStages(1));
  public static final ControlMode CONTROL_MODE = ControlMode.CLOSED_LOOP;
  public static final TelemetryVerbosity TELEMETRY = TelemetryVerbosity.HIGH;
  public static final Current STATOR_CURRENT = Amps.of(0);
  public static final Time CLOSED_RAMP_RATE = Seconds.of(0);
  public static final Time OPEN_RAMP_RATE = Seconds.of(0);
  public static final Mass MASS = Kilograms.of(0);
  public static final Distance SIZE = Meters.of(0);

  public static class Velocity {
    public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(0);
    public static final AngularAcceleration ACCELERATION = DegreesPerSecondPerSecond.of(0);
  }

  public static class Position {
    public static final boolean MOTOR_INVERTED = false;
    public static final double ANGLE_ERROR_ALLOWED = 0;
    public static final Angle MIN_ANGLE = Degrees.of(0);
    public static final Angle MAX_ANGLE = Degrees.of(0);
    public static final Angle STARTING_POSITION = Degrees.of(0);
  }

  public static class ClosedLoopController {
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static class SIM {
      public static final double kP = 100;
      public static final double kI = 0;
      public static final double kD = 0;
    }
  }

  public static class FeedForward {
    public static final double kS = 0.5;
    public static final double kV = 0.5;
    public static final double kA = 0;

    public static class SIM {
      public static final double kS = 0.5;
      public static final double kV = 0.5;
      public static final double kA = 0;
    }
  }
}
