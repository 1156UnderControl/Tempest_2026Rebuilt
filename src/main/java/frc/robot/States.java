package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.hopper.kicker.KickerSubsystem;
import frc.robot.subsystems.hopper.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.intake.extensor.ExtensorSubsystem;
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
  private final ExtensorSubsystem extensor;
  private final SpindexerSubsystem spindexer;
  private final KickerSubsystem kicker;

  private boolean isHubMode;

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

  public States(
      SwerveSubsystem drive,
      TurretSubsystem turret,
      HoodSubsystem hood,
      FlyWheelSubsystem flyWheel,
      ExtensorSubsystem extensor,
      SpindexerSubsystem spindexer,
      KickerSubsystem kicker) {
    this.drive = drive;
    this.extensor = extensor;
    this.spindexer = spindexer;
    this.kicker = kicker;
    this.turret = turret;
    this.hood = hood;
    this.flyWheel = flyWheel;

    this.isHubMode = true;

    this.turretState = "";
    this.hoodState = "";
    this.flyWheelState = "";
    this.extensorState = "";
    this.spindexerState = "";
    this.kickerState = "";
  }

  public void setDefaultStates() {
    this.drive.setDefaultCommand(
        Commands.run(() -> drive.driveAlignAngleJoystick(), drive)
            .onlyIf(() -> DriverStation.isTeleopEnabled()));

    this.kicker.setDefaultCommand(
        this.kicker.stop().alongWith(Commands.runOnce(() -> this.kickerState = "Stopped State")));
    this.spindexer.setDefaultCommand(
        this.spindexer
            .stop()
            .alongWith(Commands.runOnce(() -> this.spindexerState = "Stopped State")));

    this.flyWheel.setDefaultCommand(
        Commands.run(() -> this.flyWheel.stoppedVelocity(), flyWheel)
            .alongWith(Commands.runOnce(() -> this.flyWheelState = "Stopped State")));
  }

  public void shootState(Trigger button) {
    new Trigger(() -> !RobotStates.getInstance().isOnNotShootZone())
        .and(button)
        .whileTrue(
            Commands.parallel(
                this.spindexer.shoot(),
                this.kicker.shoot(),
                Commands.runOnce(
                    () -> {
                      this.flyWheelState =
                          this.isHubMode ? "Shooting to Hub State" : "Shooting to Pass State";
                      this.kickerState = "Running State";
                      this.spindexerState = "Running State";
                    })));
  }
}
