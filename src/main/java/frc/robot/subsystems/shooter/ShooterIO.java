package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double shooterFieldHeading = 0;
    public double shooterTargetFieldAngle = 0;

    public double wheelsCurrentVelocity = 0;
    public double wheelsTargetVelocity = 0;

    public boolean isAtTargetVelocity = false;
    public boolean isAtTargetHeading = false;

    public double distanceToTargetAngle = Double.NaN;
  }

  public void setTargetMode(boolean isHub);

  public void prepareToShoot();

  public boolean isShooterPrepared();

  public void shootFuel();

  public SubsystemBase getCurrentSubsystem();
}
