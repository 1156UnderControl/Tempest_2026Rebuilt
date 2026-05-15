package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.GoToClimbTeleop;
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
    this.setSwerveDefaultStates();
    this.setIntakeDefaultStates();
    this.setHopperDefaultStates();
    this.setShooterDefaultStates();
    // this.setClimberDefaultStates();
  }

  public Command goToClimb() {
    return new GoToClimbTeleop(this.drive);
  }

  public Command driveSwerveSlow() {
    return Commands.run(() -> this.drive.driveAlignAngleJoystickSuperSlow(), this.drive);
  }

  public Command driveSwerveNormal() {
    return Commands.run(() -> drive.driveAlignAngleJoystick(), drive)
        .alongWith(Commands.runOnce(() -> this.driveState = "Drive Align Joystick State"));
  }

  public Command shootState() {
    return Commands.parallel(this.kicker.shoot(), this.spindexer.shoot())
        .alongWith(
            Commands.runOnce(
                () -> {
                  this.kickerState = "Running State";
                  this.spindexerState = "Running State";
                }));
  }

  public void enableNoNOdometryShootState() {
    RobotStates.getInstance().setNonOdometryMode(true);
  }

  public void disableNoNOdometryShootState() {
    RobotStates.getInstance().setNonOdometryMode(false);
  }

  public Command collectState() {
    return this.extensor
        .extend()
        .andThen(
            Commands.parallel(
                this.extensor.springyMode(),
                this.roller.runRoller(),
                Commands.runOnce(
                    () -> {
                      this.extensorState = "Extended State";
                      this.rollerState = "Running State";
                    })));
  }

  public Command retractState() {
    return Commands.parallel(
        this.extensor.retract(),
        this.roller.stopRoller(),
        Commands.runOnce(
            () -> {
              this.extensorState = "Retracted State";
              this.rollerState = "Stopped State";
            }));
  }

  public Command shakeState() {
    return Commands.parallel(
        this.extensor.shake(),
        this.roller.runRoller(),
        Commands.runOnce(
            () -> {
              this.extensorState = "Shake State";
              this.rollerState = "Running State";
            }));
  }

  public Command climbState() {
    return Commands.parallel(
        this.climber.climb(),
        Commands.run(() -> this.drive.driveAlignAngleJoystickSuperSlow()),
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

  public Command expellIntakeState() {
    return this.roller.expellRoller();
  }

  public Command expellKickerState() {
    return this.kicker.expell();
  }

  private void setSwerveDefaultStates() {
    this.drive.setDefaultCommand(this.driveSwerveNormal());
  }

  private void setIntakeDefaultStates() {
    // this.extensor.setDefaultCommand(
    //     Commands.either(
    //         Commands.parallel(
    //             this.collectState(), Commands.runOnce(() -> this.extensorState = "Extended
    // State")),
    //         Commands.parallel(
    //             this.retractState(),
    //             Commands.runOnce(() -> this.extensorState = "Retracted State")),
    //         () -> RobotStates.getInstance().isExtendedMode()));
    this.roller.setDefaultCommand(this.roller.stopRoller());
  }

  private void setHopperDefaultStates() {
    this.kicker.setDefaultCommand(
        Commands.parallel(
            this.kicker.stop(), Commands.runOnce(() -> this.kickerState = "Stop State")));

    this.spindexer.setDefaultCommand(this.spindexer.stop());

  }

  private void setShooterDefaultStates() {
    this.turret.setDefaultCommand(
        Commands.parallel(
            Commands.either(
                this.turret.trackHub(),
                this.turret.trackPass(),
                () -> RobotStates.getInstance().isOnAllianceZone()),
            Commands.runOnce(() -> this.turretState = "Tracking Hub")));
    this.hood.setDefaultCommand(
        Commands.parallel(
            Commands.either(
                this.hood.trackHub(),
                this.hood.trackPass(),
                () -> RobotStates.getInstance().isOnAllianceZone()),
            Commands.runOnce(() -> this.hoodState = "Tracking Hub")));
    this.flyWheel.setDefaultCommand(
        Commands.parallel(
            this.flyWheel.stoppedVelocity(),
            Commands.runOnce(() -> this.flyWheelState = "Running Low Velocity")));

    new Trigger(
            () ->
                RobotStates.getInstance().isOnAllianceZone()
                    && !RobotStates.getInstance().isOnNonOdometryMode())
        .onTrue(Commands.parallel(this.turret.trackHub(), this.hood.trackHub()))
        .onFalse(Commands.parallel(this.turret.trackPass(), this.hood.trackPass()));

    new Trigger(() -> RobotStates.getInstance().isOnNonOdometryMode())
        .onTrue(Commands.parallel(this.turret.trackNonOdometry(), this.hood.trackNonOdometry()));
  }

  public Command calibrationState() {
    return Commands.parallel(
        this.hood.goToTargetAngleCalibration(),
        this.flyWheel.runVelocityTorqueControlCalibration());
  }
}
