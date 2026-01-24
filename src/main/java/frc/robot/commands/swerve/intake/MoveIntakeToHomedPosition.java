package frc.robot.commands.swerve.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class MoveIntakeToHomedPosition extends Command{
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();



  public MoveIntakeToHomedPosition() {
  }

  @Override
  public void initialize() {
    intakeSubsystem.goToHomedPosition();
  }

  @Override
  public void execute() {
    intakeSubsystem.goToHomedPosition();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
