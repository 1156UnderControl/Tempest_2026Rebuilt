package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class PassThroughTrenchLeftAlliance extends Command {

  SwerveSubsystem swerve;
  boolean hasAchievedPosition;

  public PassThroughTrenchLeftAlliance(SwerveSubsystem swerve) {
    this.swerve = swerve;
    this.hasAchievedPosition = false;
  }

  @Override
  public void initialize() {
    this.hasAchievedPosition = false;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    this.swerve.driveAlignAngleJoystickX(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
