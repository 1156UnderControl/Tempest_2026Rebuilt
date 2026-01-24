package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public class ClimberConstants {
  public static final int ID_climberMotor = 13;

  public static final double VELOCITY_M_PER_SECOND = 4;
  public static final double ACCELERATION_M_PER_SECOND_SQRD = 4;

  public static final double STATOR_CURRENT_LIMIT = 60;

  public class setpoints {
    public static final Angle CLIMB_ANGLE = Rotations.of(12.4);
    public static final Angle CLIMBED_ANGLE = Rotations.of(0);
    public static final Angle DEFAULT_ANGLE = Rotations.of(0);
  }

  public class PID {
    public static final double kP = 3;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double IZone = 0;
  }

  public class feedforward {
    public static final double kS = 0;
    public static final double kA = 0;
    public static final double kV = 0;
    public static final double period = 0.2;
  }
}
