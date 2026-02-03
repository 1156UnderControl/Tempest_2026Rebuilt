package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.hopper.indexer.SpindexerSubsystem;
import frc.robot.subsystems.hopper.kicker.KickerSubsystem;
import frc.robot.subsystems.intake.extensor.ExtensorSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlyWheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class States {
  private SwerveSubsystem drive;
  private final TurretSubsystem turret;
  private final HoodSubsystem hood;
  private final FlyWheelSubsystem flyWheel;
  private final ExtensorSubsystem extensor;
  private final SpindexerSubsystem spindexer;
  private final KickerSubsystem kicker;

  private boolean isHubMode;

  @AutoLogOutput private String turretState;
  @AutoLogOutput private String hoodState;
  @AutoLogOutput private String flyWheelState;
  @AutoLogOutput private String extensorState;
  @AutoLogOutput private String spindexerState;
  @AutoLogOutput private String kickerState;

  public States(SwerveSubsystem drive, TurretSubsystem turret, HoodSubsystem hood, FlyWheelSubsystem flyWheel, 
    ExtensorSubsystem extensor, SpindexerSubsystem spindexer, KickerSubsystem kicker) {
    this.drive = drive;
    this.extensor = extensor;
    this.spindexer = spindexer;
    this.kicker = kicker;
    this.turret = turret;
    this.hood = hood;
    this.flyWheel = flyWheel;

    this.isHubMode = true;
  }

  public void setDefaultStates() {
    this.drive.setDefaultCommand(
        Commands.run(() -> drive.driveAlignAngleJoystick(), drive)
            .onlyIf(() -> DriverStation.isTeleopEnabled()));

    this.extensor.setDefaultCommand(Commands.run(() -> this.extensor.holdPID(), extensor));
    this.kicker.setDefaultCommand(Commands.run(() -> this.kicker.setVelocity(RotationsPerSecond.of(0)), this.kicker));
    this.spindexer.setDefaultCommand(Commands.run(() -> this.spindexer.setVelocity(RotationsPerSecond.of(0)), this.spindexer));

    this.turret.setDefaultCommand(Commands.run(this.isHubMode ? () -> this.turret.trackHub() : () -> this.turret.trackPass(), turret).alongWith(Commands.runOnce(() -> this.turretState = isHubMode ? "Aiming at Hub Pose State" : "Aiming to Feed Pose State")));
    this.hood.setDefaultCommand(Commands.run(() -> this.hood.goToTargetAngle() ,hood).alongWith(Commands.runOnce(() -> this.hoodState = isHubMode ? "Aiming at Hub Pose State" : "Aiming to Feed Pose State")));
    this.flyWheel.setDefaultCommand(Commands.run(() -> this.flyWheel.runVelocity(), flyWheel).alongWith(Commands.runOnce(() -> this.flyWheelState = "Stopped State")));
  }

  public Command shootState() {
    this.flyWheelState =
        this.isHubMode
            ? (this.turret.isAtTargetAngle()
                    && this.hood.isAtTargetAngle()
                    && this.flyWheel.isAtTargetVelocity()
                ? "Shooting at Hub"
                : "Preparing to Shoot at Hub")
            : (this.turret.isAtTargetAngle()
                    && this.hood.isAtTargetAngle()
                    && this.flyWheel.isAtTargetVelocity())
                ? "Shooting at Alliance Zone"
                : "Preparing to Shoot at Alliance Zone";
    return Commands.parallel(Commands.run(() -> this.spindexer.setVelocity(RotationsPerSecond.of(50)), this.spindexer), Commands.run(() -> this.kicker.setVelocity(RotationsPerSecond.of(-50)), this.kicker), Commands.parallel(Commands.run(() -> this.flyWheel.runVelocity(), flyWheel)));
  }
}
