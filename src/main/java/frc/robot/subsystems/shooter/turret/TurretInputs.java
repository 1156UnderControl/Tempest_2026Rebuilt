package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class TurretInputs {
  public double currentAngle = 0;
  public double targetAngle = 0;
  public double targetVelocity = 0;
  public boolean atTarget = false;
  public double distanceToTarget = Double.NaN;
  public double absoluteAngle;
  public Pose2d turret;
  public double encoder1;
  public double encoder2;
}
