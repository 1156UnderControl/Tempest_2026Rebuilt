package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.swerve.GoToClimbPoseLeft;
import frc.robot.commands.swerve.GoToClimbPoseRight;
import frc.robot.commands.swerve.odometry.ResetOdometryBlue;
import frc.robot.commands.swerve.odometry.ResetOdometryRed;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class NamedCommandsRegistery {
  SwerveSubsystem drive;

  public NamedCommandsRegistery(SwerveSubsystem drive) {
    this.drive = drive;
  }

  public void registerAllAutoCommands() {
    this.registerGoToPoseCommands();
    this.registerResetOdometryCommands();
  }

  private void registerGoToPoseCommands() {
    NamedCommands.registerCommand("Go To Climb Left Pose", new GoToClimbPoseLeft(drive));
    NamedCommands.registerCommand("Go To Climb Right Pose", new GoToClimbPoseRight(drive));
  }

  private void registerResetOdometryCommands() {
    NamedCommands.registerCommand(
        "Reset Odometry Alliance",
        DriverStation.getAlliance().isPresent()
            ? (DriverStation.getAlliance().get() == Alliance.Blue
                ? new ResetOdometryBlue(drive)
                : new ResetOdometryRed(drive))
            : new ResetOdometryRed(drive));
  }
}
