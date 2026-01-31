package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class FlyWheelInputs {
  public double currentVelocity = 0;
  public double targetVelocity = 0;
  public boolean atTarget = false;
}
