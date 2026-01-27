package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class FlyWheelInputs {
  public double currentVelocityLead = 0;
  public double targetVelocityLead = 0;
  public boolean atTargetLead = false;

  public double currentVelocityFollower = 0;
}
