package frc.robot.constants.shooter;

import edu.wpi.first.math.system.plant.DCMotor;

public class FlyWheelsConstants {
  public static final int FLY_WHEELS_LEAD_MOTOR_ID = 0;
  public static final int FLY_WHEELS_FOLLOWER_MOTOR_ID = 0;
  public static final DCMotor FLYWHEELS_MOTOR = DCMotor.getNeoVortex(2);
  public static final double MOMENT_OF_INERTIA = 0.00017;
  public static final double GEARBOX = 1;
  public static final double VELOCITY_ERROR_ALLOWED = 0.1;
}