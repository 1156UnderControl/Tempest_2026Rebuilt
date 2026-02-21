package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class PassThroughTrenchLeftAlliance extends Command {

  SwerveSubsystem swerve;

  public PassThroughTrenchLeftAlliance(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(!this.swerve.isAtTargetSafeAllianceLeftTrenchPose()){
      this.swerve.goToTargetSafeAllianceLeftTrench();
    } else {
      this.swerve.driveAlignAngleJoystickX();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
