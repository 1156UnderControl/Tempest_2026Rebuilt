package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotStates;
import frc.robot.subsystems.hopper.kicker.KickerSubsystem;
import frc.robot.subsystems.hopper.spindexer.SpindexerSubsystem;

public class ShootCommandHub extends Command {

  private final KickerSubsystem kicker;
  private final SpindexerSubsystem spindexer;

  public ShootCommandHub(KickerSubsystem kicker, SpindexerSubsystem spindexer) {
    this.kicker = kicker;
    this.spindexer = spindexer;
  }

  @Override
  public void initialize() {
    addRequirements(this.kicker, spindexer);
  }

  @Override
  public void execute() {
    if (RobotStates.getInstance().isOnAllianceZone()) {
      Commands.parallel(this.kicker.shoot(), this.spindexer.shoot());
    } else {
      Commands.parallel(this.kicker.stop(), this.spindexer.stop());
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
