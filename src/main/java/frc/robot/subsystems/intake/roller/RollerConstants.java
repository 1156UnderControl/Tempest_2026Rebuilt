package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;

public class RollerConstants {
  public static final int ID_intakeLeaderRollerMotor = 2;
  public static final int ID_intakeFollowerRollerMotor = 3;
  public static final double GEARING = 1.0;

  public static final Current STATOR_LIMIT_TELEOP = Amps.of(55);
  public static final Current SUPPLY_LIMIT_TELEOP = Amps.of(40);

  public static final Current STATOR_LIMIT_AUTO = Amps.of(70);
  public static final Current SUPPLY_LIMIT_AUTO = Amps.of(50);

  public static final Current STATOR_LIMIT_SHOOTING = Amps.of(60);
  public static final Current SUPPLY_LIMIT_SHOOTING = Amps.of(30);
}
