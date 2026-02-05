package frc.robot.subsystems.shooter.flywheel;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class FlyWheelInputsAutoLogged extends FlyWheelInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("CurrentVelocity", currentVelocity);
    table.put("TargetVelocity", targetVelocity);
    table.put("AtTarget", atTarget);
  }

  @Override
  public void fromLog(LogTable table) {
    currentVelocity = table.get("CurrentVelocity", currentVelocity);
    targetVelocity = table.get("TargetVelocity", targetVelocity);
    atTarget = table.get("AtTarget", atTarget);
  }

  public FlyWheelInputsAutoLogged clone() {
    FlyWheelInputsAutoLogged copy = new FlyWheelInputsAutoLogged();
    copy.currentVelocity = this.currentVelocity;
    copy.targetVelocity = this.targetVelocity;
    copy.atTarget = this.atTarget;
    return copy;
  }
}
