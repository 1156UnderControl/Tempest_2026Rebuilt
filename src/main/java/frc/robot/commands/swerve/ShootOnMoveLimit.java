package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ShootOnMoveLimit extends Command {

  SwerveSubsystem swerve;

  public ShootOnMoveLimit(SwerveSubsystem swerve) {
    this.swerve = swerve;
  }

  @Override
  public void initialize() {
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    this.swerve.driveAlignAngleJoystickSuperSlow();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
