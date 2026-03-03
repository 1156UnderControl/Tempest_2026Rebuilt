package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class PassThroughBump extends Command {

  SwerveSubsystem swerve;
  Rotation2d lockedAngle;

  public PassThroughBump(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    this.lockedAngle = swerve.getNearestBumpAngle();
  }

  @Override
  public void execute() {
    this.swerve.driveAlignBump(lockedAngle);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
