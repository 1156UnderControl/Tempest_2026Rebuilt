package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class PassThroughTower extends Command {

  SwerveSubsystem swerve;

  public PassThroughTower(SwerveSubsystem swerve) {
    this.swerve = swerve;
  }

  @Override
  public void initialize() {
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    this.swerve.driveAlignAngleJoystickYfixedOnTower();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}