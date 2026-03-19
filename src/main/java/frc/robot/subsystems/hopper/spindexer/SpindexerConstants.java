package frc.robot.subsystems.hopper.spindexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public final class SpindexerConstants {
  public static final int MOTOR_ID = 6;
  public static final Current STATOR_CURRENT_LIMIT = Amps.of(65);
  public static final Time CLOSED_LOOP_RAMP = Milliseconds.of(10);
  public static final Voltage MAX_VOLTS = Volts.of(12);
  public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(5.6);
  public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(60);

  public static final AngularVelocity DEADBAND_IS_AT = RPM.of(10);

  public static class setpoints {
    public static final AngularVelocity SHOOTING = RotationsPerSecond.of(10);
    public static final AngularVelocity EXPELLING = RotationsPerSecond.of(-1.0);
    public static final AngularVelocity STOPPED = RPM.of(0);
  }

  public static final double FEEDFORWARD_TORQUE_CURRENT_A = 3;

}
