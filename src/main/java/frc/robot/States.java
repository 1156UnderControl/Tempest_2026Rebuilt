package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.hopper.kicker.KickerSubsystem;
import frc.robot.subsystems.hopper.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.intake.extensor.ExtensorSubsystem;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlyWheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;

public class States {
  private SwerveSubsystem drive;
  private final TurretSubsystem turret;
  private final HoodSubsystem hood;
  private final FlyWheelSubsystem flyWheel;
  private final SpindexerSubsystem spindexer;
  private final KickerSubsystem kicker;
  private final RollerSubsystem roller;
  private final ExtensorSubsystem extensor;

  @AutoLogOutput(key = "States/TurretStates")
  private String turretState;

  @AutoLogOutput(key = "States/HoodStates")
  private String hoodState;

  @AutoLogOutput(key = "States/FlyWheelStates")
  private String flyWheelState;

  @AutoLogOutput(key = "States/ExtensorStates")
  private String extensorState;

  @AutoLogOutput(key = "States/SpindexerStates")
  private String spindexerState;

  @AutoLogOutput(key = "States/KickersStates")
  private String kickerState;

  @AutoLogOutput(key = "States/RollerState")
  private String rollerState;

  public States(
      SwerveSubsystem drive,
      TurretSubsystem turret,
      HoodSubsystem hood,
      FlyWheelSubsystem flyWheel,
      SpindexerSubsystem spindexer,
      KickerSubsystem kicker,
      RollerSubsystem roller,
      ExtensorSubsystem extensor) {
    this.roller = roller;
    this.drive = drive;
    this.spindexer = spindexer;
    this.kicker = kicker;
    this.turret = turret;
    this.hood = hood;
    this.flyWheel = flyWheel;
    this.extensor = extensor;

    this.turretState = "";
    this.hoodState = "";
    this.flyWheelState = "";
    this.extensorState = "";
    this.spindexerState = "";
    this.kickerState = "";
    this.rollerState = "";
  }

  public void setDefaultStates() {
    this.drive.setDefaultCommand(
        Commands.run(() -> drive.driveAlignAngleJoystick(), drive)
            .onlyIf(() -> DriverStation.isTeleopEnabled()));

    this.kicker.setDefaultCommand(
        this.kicker
        .stop()
        .alongWith(Commands.runOnce(() -> this.kickerState = "Stopped State")));

    this.spindexer.setDefaultCommand(
        this.spindexer
            .stop()
            .alongWith(Commands.runOnce(() -> this.spindexerState = "Stopped State")));

    this.turret.setDefaultCommand(Commands.either(
      this.turret.trackHub().alongWith(Commands.runOnce(() ->
      this.turretState = "Track Hub State")), 
      this.turret.trackPass().alongWith(Commands.runOnce(() ->
      this.turretState = "Track Pass State")), 
      RobotStates.getInstance().isOnAllianceZone()));

    this.flyWheel.setDefaultCommand(this.flyWheel.runVelocity()
      .alongWith(Commands.runOnce(() -> this.flyWheelState = "Running State")));
  }

  public Command shootState() {
    return Commands.parallel(
      this.kicker.shoot(), 
      this.spindexer.shoot(),
      this.extensor.shake()
      ).alongWith(
        Commands.runOnce(
          () -> {
            this.kickerState = "Running State";
            this.spindexerState = "Running State";
            this.extensorState = "Shake State";
          }
        ));
  }

  public Command collectState() {
    return this.extensor.extend().alongWith(this.roller.runRoller(), Commands.runOnce(()-> {
      this.extensorState = "Extended State";
      this.rollerState = "Running State";
    })).onlyWhile(this.extensor.isAtCollision().negate()).andThen(this.extensor.softStopExtensor().alongWith(this.roller.stopRoller(), Commands.runOnce(()-> {
      this.extensorState = "Bumped State";
      this.rollerState = "Bumped State";
  }))).repeatedly();
  }

  public Command retractState() {
    return this.extensor.retract().alongWith(this.roller.stopRoller(), Commands.runOnce(()-> {
      this.extensorState = "Retracted State";
      this.rollerState = "Stopped State";
    }));
  }
}
