package frc.robot.constants.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

/** Constants describing the hopper's spindexer wheel. */
public final class SpindexerConstants {
  public static final int MOTOR_ID = 1;
  public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
  public static final Time CLOSED_LOOP_RAMP = Seconds.of(0.25);
  public static final Time OPEN_LOOP_RAMP = Seconds.of(0.25);
  public static final Voltage MAX_VOLTS = Volts.of(12);

  public static final AngularVelocity DEADBAND_IS_AT = RPM.of(100);

  public static class setpoints {
    public static final AngularVelocity SHOOTING = RPM.of(3000);
    public static final AngularVelocity STOPPED = RPM.of(0);
  }

  public static class ClosedLoopController {
    public static final double kP = 100;
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
