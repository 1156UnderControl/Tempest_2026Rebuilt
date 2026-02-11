package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.ClimberSubsystem;
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
  private final ClimberSubsystem climber;

  @AutoLogOutput(key = "States/DriveState")
  private String driveState;

  @AutoLogOutput(key = "States/TurretState")
  private String turretState;

  @AutoLogOutput(key = "States/HoodState")
  private String hoodState;

  @AutoLogOutput(key = "States/FlyWheelState")
  private String flyWheelState;

  @AutoLogOutput(key = "States/ExtensorState")
  private String extensorState;

  @AutoLogOutput(key = "States/SpindexerState")
  private String spindexerState;

  @AutoLogOutput(key = "States/KickersState")
  private String kickerState;

  @AutoLogOutput(key = "States/RollerState")
  private String rollerState;

  @AutoLogOutput(key = "States/ClimberState")
  private String climberState;

  public States(
      SwerveSubsystem drive,
      TurretSubsystem turret,
      HoodSubsystem hood,
      FlyWheelSubsystem flyWheel,
      SpindexerSubsystem spindexer,
      KickerSubsystem kicker,
      RollerSubsystem roller,
      ExtensorSubsystem extensor,
      ClimberSubsystem climber) {
    this.roller = roller;
    this.drive = drive;
    this.spindexer = spindexer;
    this.kicker = kicker;
    this.turret = turret;
    this.hood = hood;
    this.flyWheel = flyWheel;
    this.extensor = extensor;
    this.climber = climber;

    this.driveState = "";
    this.turretState = "";
    this.hoodState = "";
    this.flyWheelState = "";
    this.extensorState = "";
    this.spindexerState = "";
    this.kickerState = "";
    this.rollerState = "";
    this.climberState = "";
  }

  public void setDefaultStates() {
    this.drive.setDefaultCommand(
        Commands.run(() -> drive.driveAlignAngleJoystick(), drive)
            .onlyIf(() -> DriverStation.isTeleopEnabled())
            .alongWith(Commands.runOnce(() -> this.driveState = "Drive Align Joystick State")));

    this.extensor.setDefaultCommand(
        this.extensor
            .retract()
            .alongWith(Commands.runOnce(() -> this.extensorState = "Retracted State")));

    this.roller.setDefaultCommand(
        this.roller
            .stopRoller()
            .alongWith(Commands.runOnce(() -> this.rollerState = "Stopped State")));

    this.kicker.setDefaultCommand(
        this.kicker.stop().alongWith(Commands.runOnce(() -> this.kickerState = "Stopped State")));

    this.spindexer.setDefaultCommand(
        this.spindexer
            .stop()
            .alongWith(Commands.runOnce(() -> this.spindexerState = "Stopped State")));

    new Trigger(() -> RobotStates.getInstance().isOnAllianceZone())
        .onTrue(this.turret.trackHub())
        .onFalse(this.turret.trackPass());

    this.hood.setDefaultCommand(this.hood.goToTargetAngle());

    this.flyWheel.setDefaultCommand(
        this.flyWheel
            .runVelocity()
            .alongWith(Commands.runOnce(() -> this.flyWheelState = "Running State")));

    this.climber.setDefaultCommand(
        this.climber
            .defaultPosition()
            .alongWith(Commands.runOnce(() -> this.climberState = "Default State")));
  }

  public Command shootState() {
    return Commands.parallel(this.kicker.shoot(), this.spindexer.shoot(), this.extensor.shake())
        .alongWith(
            Commands.runOnce(
                () -> {
                  this.kickerState = "Running State";
                  this.spindexerState = "Running State";
                  this.extensorState = "Shake State";
                }))
        .repeatedly();
  }

  public Command collectState() {
    return this.extensor
        .extend()
        .alongWith(
            this.roller.runRoller(),
            Commands.runOnce(
                () -> {
                  this.extensorState = "Extended State";
                  this.rollerState = "Running State";
                }))
        .onlyWhile(this.extensor.isAtCollision().negate())
        .andThen(
            this.extensor
                .softStopExtensor()
                .alongWith(
                    this.roller.stopRoller(),
                    Commands.runOnce(
                        () -> {
                          this.extensorState = "Bumped State";
                          this.rollerState = "Bumped State";
                        })))
        .repeatedly();
  }

  public Command retractState() {
    return this.extensor
        .retract()
        .alongWith(
            this.roller.stopRoller(),
            Commands.runOnce(
                () -> {
                  this.extensorState = "Retracted State";
                  this.rollerState = "Stopped State";
                }))
        .repeatedly();
  }

  public Command climbState() {
    return Commands.parallel(
        Commands.run(() -> drive.driveHeadingBackwards(), drive),
        this.climber.climb(),
        Commands.runOnce(
            () -> {
              this.climberState = "Climbed State";
              this.driveState = "At Climb Pose State Right";
            }));
  }

  public Command defaultClimberState() {
    return this.climber
        .defaultPosition()
        .alongWith(
            Commands.runOnce(
                () -> {
                  this.climberState = "Climbed State";
                  this.driveState = "At Climb Pose State Right";
                }));
  }
}
