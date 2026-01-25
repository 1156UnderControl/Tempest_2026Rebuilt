package frc.robot.constants.shooter;

import edu.wpi.first.math.system.plant.DCMotor;

public class HoodConstants {
  public static final int HOOD_MOTOR_ID = 1;
  public static final double HOOD_MIN_ANGLE = -200;
  public static final double HOOD_MAX_ANGLE = 200;
  public static final DCMotor HOOD_MOTOR = DCMotor.getNEO(1);
  public static final double MOMENT_OF_INERTIA = 0.08;
  public static final double GEARBOX = 360;
  public static final double ANGLE_ERROR_ALLOWED = 5;
}
