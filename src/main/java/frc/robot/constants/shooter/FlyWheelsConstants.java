package frc.robot.constants.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class FlyWheelsConstants {
  public static final int FLYWHEEL_MOTOR_LEAD_ID = 19;
  public static final int FLYWHEEL_MOTOR_FOLLOWER_ID = 18;
  public static final String MOTOR_LEAD_NAME = "FlyWheelLeadMotor";
  public static final String MOTOR_FOLLOWER_NAME = "FlyWheelFollowerMotor";
  public static final DCMotor FLYWHEEL_MOTOR = DCMotor.getKrakenX44(2);
  public static final MotorMode MOTOR_MODE = MotorMode.BRAKE;
  public static final MechanismGearing MOTOR_GEARING =
      new MechanismGearing(GearBox.fromReductionStages(1));
  public static final ControlMode CONTROL_MODE = ControlMode.OPEN_LOOP;
  public static final TelemetryVerbosity TELEMETRY = TelemetryVerbosity.HIGH;
  public static final Current STATOR_CURRENT = Amps.of(60);
  public static final Time CLOSED_RAMP_RATE = Seconds.of(0.5);
  public static final Time OPEN_RAMP_RATE = Seconds.of(0.5);
  public static final Mass FLYWHEEL_MASS = Kilograms.of(1);
  public static final Distance FLYWHEEL_SIZE = Meters.of(0.11);
  public static final Voltage MAX_VOLTS = Volts.of(12.0);

  public static class Velocity {
    public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(83);
    public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(40);
    public static final double VELOCITY_ERROR_ALLOWED = 0.01;
  }

  public static class Position {
    public static final boolean MOTOR_INVERTED_LEAD = false;
    public static final boolean MOTOR_INVERTED_FOLLOWER = true;
  }

  public static class ClosedLoopController {
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static class SIM {
      public static final double kP = 1;
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
