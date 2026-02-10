package frc.robot.constants.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class TurretConstants {
  public static final TelemetryVerbosity TELEMETRY = TelemetryVerbosity.HIGH;
  public static final Voltage MAX_VOLTS = Volts.of(12.0);
  public static final int TURRET_GEAR_TEETH = 96;
  public static final double PHASE_DELAY = 0.03;

  public static class Velocity {
    public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(360);
    public static final AngularAcceleration ACCELERATION = DegreesPerSecondPerSecond.of(200);
  }

  public static class Position {
    public static final Transform2d TURRET_OFFSET_FROM_ROBOT_CENTER =
        new Transform2d(-0.127, -0.18415, Rotation2d.kZero);
    public static final Angle ANGLE_ERROR_ALLOWED = Degrees.of(1.0);
    public static final Angle TURRET_MIN_ANGLE = Degrees.of(-180);
    public static final Angle TURRET_MAX_ANGLE = Degrees.of(180);
    public static final Angle TURRET_STARTING_ANGLE = Degrees.of(0);
  }

  public static class ClosedLoopController {
    public static final double kP = 100;
    public static final double kI = 0;
    public static final double kD = 0;
    public static class SIM {
      public static final double kP = 200;
      public static final double kI = 0;
      public static final double kD = 0;
    }
  }
}
