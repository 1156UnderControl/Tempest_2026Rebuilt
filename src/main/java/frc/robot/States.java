package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.indexer.SpindexerSubsystem;
import frc.robot.subsystems.hopper.kicker.KickerSubsystem;
import frc.robot.subsystems.intake.extensor.ExtensorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class States {
  private SwerveSubsystem drive;
  private final ExtensorSubsystem extensor;
  private final SpindexerSubsystem spindexer;
  private final KickerSubsystem kicker;

  // private boolean isHubMode;
  // @AutoLogOutput private String turretState;
  // @AutoLogOutput private String hoodState;
  // @AutoLogOutput private String flyWheelState;

  // public States(
  //     SwerveSubsystem drive,
  //     TurretSubsystem turret,
  //     HoodSubsystem hood,
  //     FlyWheelSubsystem flyWheel) {
  //   this.isHubMode = true;
  //   this.drive = drive;
  //   this.turret = turret;
  //   this.hood = hood;
  //   this.flyWheel = flyWheel;
  // }

  public States(
      SwerveSubsystem drive,
      ExtensorSubsystem extensor,
      SpindexerSubsystem spindexer,
      KickerSubsystem kicker) {
    this.drive = drive;
    this.extensor = extensor;
    this.spindexer = spindexer;
    this.kicker = kicker;
    // this.isHubMode = true;
    // this.turret = turret;
    // this.hood = hood;
    // this.flyWheel = flyWheel;
  }

  public void setDefaultStates() {
    this.drive.setDefaultCommand(
        Commands.run(() -> drive.driveAlignAngleJoystick(), drive)
            .onlyIf(() -> DriverStation.isTeleopEnabled()));

    this.extensor.setDefaultCommand(Commands.run(() -> this.extensor.holdPID(), extensor));
    this.kicker.setDefaultCommand(
        Commands.run(() -> this.kicker.setVelocity(RotationsPerSecond.of(0)), this.kicker));
    this.spindexer.setDefaultCommand(
        Commands.run(() -> this.spindexer.setVelocity(RotationsPerSecond.of(0)), this.spindexer));

    // this.turretState = isHubMode ? "Aiming at Hub Pose State" : "Aiming to Feed Pose State";
    // this.turret.setDefaultCommand(this.turret.goToTargetAngle());

    // this.hoodState = isHubMode ? "Aiming at Hub Pose State" : "Aiming to Feed Pose State";
    // this.hood.setDefaultCommand(Commands.run(() -> this.hood.goToTargetAngle(), this.hood));

    // this.flyWheelState = "Stopped State";
    // this.flyWheel.setDefaultCommand(Commands.run(() -> this.flyWheel.runVelocity(), flyWheel));
  }

  public Command shootState() {
    return Commands.race(
        Commands.run(() -> this.spindexer.setVelocity(RotationsPerSecond.of(50)), this.spindexer),
        Commands.run(() -> this.kicker.setVelocity(RotationsPerSecond.of(-50)), this.kicker));
    //   this.flyWheelState =
    //       isHubMode
    //           ? (this.turret.isAtTargetAngle()
    //                   && this.hood.isAtTargetAngle()
    //                   && this.flyWheel.isAtTargetVelocity()
    //               ? "Shooting at Hub"
    //               : "Preparing to Shoot at Hub")
    //           : (this.turret.isAtTargetAngle()
    //                   && this.hood.isAtTargetAngle()
    //                   && this.flyWheel.isAtTargetVelocity())
    //               ? "Shooting at Alliance Zone"
    //               : "Preparing to Shoot at Alliance Zone";
    //   return Commands.parallel(Commands.run(() -> this.flyWheel.runVelocity(), flyWheel));
  }
}
