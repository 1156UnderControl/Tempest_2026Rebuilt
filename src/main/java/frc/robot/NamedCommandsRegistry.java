package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.swerve.GoToClimbPoseLeft;
import frc.robot.commands.swerve.GoToClimbPoseRight;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class NamedCommandsRegistry {
  SwerveSubsystem drive;

  public NamedCommandsRegistry(SwerveSubsystem drive) {
    this.drive = drive;
  }

  public void registerAllAutoCommands() {
    this.registerGoToPoseCommands();
  }

  private void registerGoToPoseCommands() {
    NamedCommands.registerCommand("Go To Climb Left Pose", new GoToClimbPoseLeft(drive));
    NamedCommands.registerCommand("Go To Climb Right Pose", new GoToClimbPoseRight(drive));
  }
}
