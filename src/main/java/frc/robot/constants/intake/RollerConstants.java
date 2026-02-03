package frc.robot.constants.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import yams.mechanisms.config.MechanismPositionConfig;

public class RollerConstants {
  public static final int ID_intakeLeaderRollerMotor = 2;
  public static final int ID_intakeFollowerRollerMotor = 3;

  public static final Distance MECHANISM_CIRCUNFERENCE = Inches.of(4);

  public static final MechanismPositionConfig robotToMechanism =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(1.5))
          .withMaxRobotLength(Meters.of(0.75))
          .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)));

  public class setpoints {
    public static final double INTAKE_DUTY_CYCLE = 0.8;
    public static final double REVERSE_DUTY_CYCLE = -0.8;
    public static final double STOP_DUTY_CYCLE = 0;
  }
}
