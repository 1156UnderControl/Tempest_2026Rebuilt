package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterCalculator;
import frc.robot.subsystems.shooter.flywheel.FlyWheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;

public class States {
  private SwerveSubsystem drive;
  private TurretSubsystem turret;
  private HoodSubsystem hood;
  private FlyWheelSubsystem flyWheel;

  private boolean isHubMode;

  @AutoLogOutput private String turretState;
  @AutoLogOutput private String hoodState;
  @AutoLogOutput private String flyWheelState;

  public States(
      SwerveSubsystem drive,
      TurretSubsystem turret,
      HoodSubsystem hood,
      FlyWheelSubsystem flyWheel) {
    this.isHubMode = true;
    this.drive = drive;
    this.turret = turret;
    this.hood = hood;
    this.flyWheel = flyWheel;
  }

  public void setDefaultStates() {
    this.drive.setDefaultCommand(
        Commands.run(() -> drive.driveAlignAngleJoystick(), drive)
            .onlyIf(() -> DriverStation.isTeleopEnabled()));

    this.turretState = isHubMode ? "Aiming at Hub Pose State" : "Aiming to Feed Pose State";
    this.turret.setDefaultCommand(Commands.run(() -> this.turret.goToTargetAngle(), turret));

    this.hoodState = isHubMode ? "Aiming at Hub Pose State" : "Aiming to Feed Pose State";
    this.hood.setDefaultCommand(Commands.run(() -> this.hood.goToTargetAngle(), this.hood));

    this.flyWheelState = "Stopped State";
    this.flyWheel.setDefaultCommand(Commands.run(() -> this.flyWheel.runVelocity(), flyWheel));
  }

  public Command shootState() {
    this.flyWheelState =
        isHubMode
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
    return Commands.parallel(Commands.run(() -> this.flyWheel.runVelocity(), flyWheel));
  }

  public Command setTargetModeCommand(boolean isHubMode) {
    this.isHubMode = isHubMode;
    return Commands.runOnce(() -> ShooterCalculator.getInstance().setHubMode(isHubMode));
  }
}
