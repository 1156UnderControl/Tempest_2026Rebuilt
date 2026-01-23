package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class GoToClimbPoseRight extends Command {

  SwerveSubsystem swerve;

  public GoToClimbPoseRight(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.swerve.goToClimbPoseRight();
  }

  @Override
  public boolean isFinished() {
    return this.swerve.isAtTargetPositionWithHeading();
  }
}
