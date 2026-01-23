package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class ClimberConstants {
  public static final int ID_climberMotor = 10;

  public static final double VELOCITY_M_PER_SECOND = 4;
  public static final double ACCELERATION_M_PER_SECOND_SQRD = 4;

  public static final double STATOR_CURRENT_LIMIT = 60;

  public class setpoints {
    public static final Angle CLIMB_ANGLE = Degrees.of(770);
    public static final Angle CLIMBED_ANGLE = Degrees.of(0);
    public static final Angle DEFAULT_ANGLE = Degrees.of(0);
  }

  public class PID {
    public static final double kP = 200;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double IZone = 0;
  }

  public class feedforward {
    public static final double kS = 200;
    public static final double kA = 0;
    public static final double kV = 0;
    public static final double kG = 0;
  }
}
