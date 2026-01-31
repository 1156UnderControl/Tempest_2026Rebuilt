package frc.robot.commands.swerve.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ResetOdometryBlue extends Command {

  SwerveSubsystem swerve;

  public ResetOdometryBlue(SwerveSubsystem swerve) {
    this.swerve = swerve;
    this.addRequirements(swerve);
  }

  @Override
  public void initialize() {
    this.swerve.resetOdometry(new Pose2d(1.0, 5.0, new Rotation2d(0)));
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
