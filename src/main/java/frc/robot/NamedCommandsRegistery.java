
package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.intake.ExtendIntake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.shooter.ShootCommandHub;
import frc.robot.commands.shooter.ShootCommandPass;
import frc.robot.commands.swerve.GoToClimbPoseLeft;
import frc.robot.commands.swerve.GoToClimbPoseRight;
import frc.robot.commands.swerve.odometry.ResetOdometryBlue;
import frc.robot.commands.swerve.odometry.ResetOdometryRed;
import frc.robot.subsystems.hopper.kicker.KickerSubsystem;
import frc.robot.subsystems.hopper.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.intake.extensor.ExtensorSubsystem;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class NamedCommandsRegistery {
  SwerveSubsystem drive;
  RollerSubsystem roller;
  ExtensorSubsystem extensor;
  KickerSubsystem kicker;
  SpindexerSubsystem spindexer;

  public NamedCommandsRegistery(SwerveSubsystem drive, RollerSubsystem roller, ExtensorSubsystem extensor, KickerSubsystem kicker, SpindexerSubsystem spindexer) {
    this.drive = drive;
    this.roller = roller;
    this.extensor = extensor;
  }

  public void registerAllAutoCommands() {
    this.registerGoToPoseCommands();
    this.registerResetOdometryCommands();
    this.registerIntakeCommands();
    this.registerShooterCommands();
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

  private void registerIntakeCommands() {
    NamedCommands.registerCommand("Extend Intake", new ExtendIntake(this.roller, this.extensor));
    NamedCommands.registerCommand("Retract Intake", new RetractIntake(this.roller, this.extensor));
  }

  private void registerShooterCommands() {
    NamedCommands.registerCommand("Shoot Fuels Hub", new ShootCommandHub(this.kicker, this.spindexer));
    NamedCommands.registerCommand("Shoot Fuels Pass", new ShootCommandPass(this.kicker, this.spindexer));
  }
}
