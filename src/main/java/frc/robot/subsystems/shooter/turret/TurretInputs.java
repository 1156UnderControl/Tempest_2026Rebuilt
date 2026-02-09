package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

@AutoLog
public class TurretInputs {
  public double currentAngle = 0;
  public double targetAngle = 0;
  public boolean atTarget = false;
  public double distanceToTarget = Double.NaN;
  public double absoluteAngle;
  public Pose2d turret;
}
