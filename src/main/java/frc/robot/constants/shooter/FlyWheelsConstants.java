package frc.robot.constants.shooter;

import static edu.wpi.first.units.Units.Amps;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
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
  public static final MechanismGearing MOTOR_GEARING = new MechanismGearing(GearBox.fromReductionStages(1));
  public static final ControlMode CONTROL_MODE = ControlMode.OPEN_LOOP;
  public static final TelemetryVerbosity TELEMETRY = TelemetryVerbosity.HIGH;
  public static final Current STATOR_CURRENT = Amps.of(60);
  public static final Time CLOSED_RAMP_RATE = Seconds.of(0.5);
  public static final Time OPEN_RAMP_RATE = Seconds.of(0.5);

  public static class Velocity {
    public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(83);
    public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(40);
    public static final double VELOCITY_ERROR_ALLOWED = 0.01;
  }

  public static class Position {
    public static final boolean MOTOR_INVERTED_LEAD = false;
    public static final boolean MOTOR_INVERTED_FOLLOWER = true;
  }

  public static class ClosedLoopController{
    public static final double P = 1;
    public static final double I = 0;
    public static final double D = 0;
  }

  public static class FeedForward{
    public static final double KS = 0.5;
    public static final double KV = 0.5;
    public static final double KA = 0;
  }
}
