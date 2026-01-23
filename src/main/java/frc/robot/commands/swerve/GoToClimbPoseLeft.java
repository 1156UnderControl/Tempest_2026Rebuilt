package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class GoToClimbPoseLeft extends Command {

  SwerveSubsystem swerve;

  public GoToClimbPoseLeft(SwerveSubsystem swerve) {
    this.swerve = swerve;
    this.addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.swerve.goToClimbPoseLeft();
  }

  @Override
  public boolean isFinished() {
    return this.swerve.isAtTargetPositionWithHeading();
  }
}
