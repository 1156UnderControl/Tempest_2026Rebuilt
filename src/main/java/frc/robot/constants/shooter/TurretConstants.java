package frc.robot.constants.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class TurretConstants {
  public static final int TURRET_MOTOR_ID = 0;
  public static final Translation2d TURRET_OFFSET_FROM_ROBOT_CENTER = new Translation2d(0, 0);
  public static final double TURRET_MIN_ANGLE = -200;
  public static final double TURRET_MAX_ANGLE = 200;
  public static final DCMotor TURRET_MOTOR = DCMotor.getKrakenX60(1);
  public static final double MOMENT_OF_INERTIA = 0.08;
  public static final double GEARBOX = 360;
  public static final double ANGLE_ERROR_ALLOWED = 5;
}
