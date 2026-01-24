package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class MoveIntakeToCollectPosition extends Command {
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  public MoveIntakeToCollectPosition() {}

  @Override
  public void initialize() {
    intakeSubsystem.goToCollectPostion();
  }

  @Override
  public void execute() {
    intakeSubsystem.goToCollectPostion();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
