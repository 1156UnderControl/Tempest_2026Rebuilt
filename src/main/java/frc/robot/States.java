package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;

public class States {
  private SwerveSubsystem drive;
  private ShooterSubsystem shooter;
  @AutoLogOutput private static String states;

  public States(SwerveSubsystem drive, ShooterSubsystem shooter) {
    this.drive = drive;
    this.shooter = shooter;
  }

  public void setDefaultStates() {
    this.drive.setDefaultCommand(
        Commands.run(() -> drive.driveAlignAngleJoystick(), drive)
            .onlyIf(() -> DriverStation.isTeleopEnabled()));
    this.shooter.setDefaultCommand(Commands.run(() -> shooter.prepareToShoot(), shooter));
  }

  public Command shootState() {
    return Commands.run(() -> this.shooter.shootFuel(), shooter);
  }
}
