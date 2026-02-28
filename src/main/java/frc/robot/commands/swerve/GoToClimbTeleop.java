package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class GoToClimbTeleop extends Command {

  SwerveSubsystem swerve;

  boolean wentToClimb = false;

  public GoToClimbTeleop(SwerveSubsystem swerve) {
    this.swerve = swerve;
    this.addRequirements(swerve);
  }

  @Override
  public void initialize() {
    this.wentToClimb = false;
  }

  @Override
  public void execute() {
    if (!this.wentToClimb && this.swerve.isAtSafezoneForGoToClimb()) {
      this.swerve.driveAlignAngleJoystick();
    } else {
      this.wentToClimb = true;
      if (this.swerve.isAtRightClimb()) {
        this.swerve.goToClimbPoseRight();
      } else {
        this.swerve.goToClimbPoseLeft();
      }
    }
  }

  @Override
  public boolean isFinished() {
    return this.swerve.isAtTargetPositionWithHeading() && this.wentToClimb;
  }

  @Override
  public void end(boolean interrupted) {
    this.swerve.stopSwerve();
  }
}
