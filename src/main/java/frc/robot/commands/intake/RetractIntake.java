package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.extensor.ExtensorSubsystem;
import frc.robot.subsystems.intake.roller.RollerSubsystem;

public class RetractIntake extends Command {

    private final RollerSubsystem roller;
    private final ExtensorSubsystem extensor;

  public RetractIntake(RollerSubsystem roller, ExtensorSubsystem extensor) {
    this.roller = roller;
    this.extensor = extensor;
  }

  @Override
  public void initialize() {
    addRequirements(this.roller, extensor);
  }

  @Override
  public void execute() {
    Commands.parallel(this.extensor.retract(), this.roller.runRoller());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
}