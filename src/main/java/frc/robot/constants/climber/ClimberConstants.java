package frc.robot.constants.climber;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import yams.mechanisms.config.MechanismPositionConfig;

public class ClimberConstants {
  public static final int ID_climberMotor = 5;

  public static final Distance DEADBAND_IS_AT = Meters.of(0.1);

  public static final double VELOCITY_M_PER_SECOND = 0;

  public static final double ACCELERATION_M_PER_SECOND_SQRD = 0;

  public static final Distance MECHANISM_CIRCUNFERENCE = Meters.of(2 * Math.PI * 0.015875);

  public static final MechanismPositionConfig robotToMechanism =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(1.5))
          .withMaxRobotLength(Meters.of(0.75))
          .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)));

  public class limits {
    public static final Distance MIN_HEIGHT = Distance.ofBaseUnits(0, Meters);
    public static final Distance MAX_HEIGHT = Distance.ofBaseUnits(12.4, Meters);
  }

  public class setpoints {
    public static final Distance RAISED_HEIGHT = Distance.ofBaseUnits(12.4, Meters);
    public static final Distance CLIMBED_HEIGHT = Distance.ofBaseUnits(0, Meters);
    public static final Distance DEFAULT_HEIGHT = Distance.ofBaseUnits(0, Meters);
  }

  public class PID {
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double F = 0;
    public static final double IZone = 0;
  }
}
