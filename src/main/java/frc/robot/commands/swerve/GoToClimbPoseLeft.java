package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class GoToClimbPoseLeft extends Command {

  SwerveSubsystem swerve;

  public GoToClimbPoseLeft(SwerveSubsystem swerve) {
    this.swerve = swerve;
    this.addRequirements(swerve);
  }

  @Override
  public void initialize() {
    this.swerve.goToClimbPoseLeft();
  }

  @Override
  public void execute() {
    this.swerve.goToClimbPoseLeft();
  }

  @Override
  public boolean isFinished() {
    return this.swerve.isAtTargetPositionWithHeading();
  }

  @Override
  public void end(boolean interrupted) {
    if (DriverStation.isAutonomous()) {
      this.swerve.stopSwerve();
    }
  }
}
