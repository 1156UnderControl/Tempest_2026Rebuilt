package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.swerve.GoToClimbPoseLeft;
import frc.robot.commands.swerve.GoToClimbPoseRight;
import frc.robot.commands.swerve.odometry.ResetOdometryBlue;
import frc.robot.commands.swerve.odometry.ResetOdometryRed;
import frc.robot.subsystems.climber.ClimberSubsystem;
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
  ClimberSubsystem climber;

  public NamedCommandsRegistery(
      SwerveSubsystem drive,
      RollerSubsystem roller,
      ExtensorSubsystem extensor,
      KickerSubsystem kicker,
      SpindexerSubsystem spindexer,
      ClimberSubsystem climber) {
    this.drive = drive;
    this.roller = roller;
    this.extensor = extensor;
    this.kicker = kicker;
    this.spindexer = spindexer;
    this.climber = climber;
  }

  public void registerAllAutoCommands() {
    this.registerGoToPoseCommands();
    this.registerResetOdometryCommands();
    this.registerIntakeCommands();
    this.registerShooterCommands();
    this.registerClimberCommands();
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
    NamedCommands.registerCommand(
        "Extend Intake", this.extensor.extend().andThen(this.roller.runRoller()));
    NamedCommands.registerCommand(
        "Retract Intake",
        this.extensor.retractAuto().alongWith(roller.stopRoller()));
  }

  private void registerShooterCommands() {
    NamedCommands.registerCommand(
        "Shoot Fuels",
        Commands.runOnce(() -> RobotStates.getInstance().setShouldShootAutonomous(true)));
    NamedCommands.registerCommand(
        "Stop Shooting Fuels",
        Commands.runOnce(() -> RobotStates.getInstance().setShouldShootAutonomous(false)));
  }

  private void registerClimberCommands() {
    NamedCommands.registerCommand("Raise Climber", this.climber.climb());
    NamedCommands.registerCommand("Default Climber", this.climber.defaultPosition());
  }
}
