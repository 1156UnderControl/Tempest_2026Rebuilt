package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.extensor.ExtensorSubsystem;
import frc.robot.subsystems.intake.roller.RollerSubsystem;

public class ExtendIntake extends Command {

    private final RollerSubsystem roller;
    private final ExtensorSubsystem extensor;

  public ExtendIntake(RollerSubsystem roller, ExtensorSubsystem extensor) {
    this.roller = roller;
    this.extensor = extensor;
  }

  @Override
  public void initialize() {
    addRequirements(this.roller, extensor);
  }

  @Override
  public void execute() {
    this.extensor.extend().andThen(this.roller.runRoller());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
}