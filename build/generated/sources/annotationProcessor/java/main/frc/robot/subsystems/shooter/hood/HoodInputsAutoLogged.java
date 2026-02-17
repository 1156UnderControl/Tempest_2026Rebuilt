package frc.robot.subsystems.shooter.hood;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class HoodInputsAutoLogged extends HoodInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("CurrentAngle", currentAngle);
    table.put("TargetAngle", targetAngle);
    table.put("AngleError", angleError);
    table.put("AtTarget", atTarget);
  }

  @Override
  public void fromLog(LogTable table) {
    currentAngle = table.get("CurrentAngle", currentAngle);
    targetAngle = table.get("TargetAngle", targetAngle);
    angleError = table.get("AngleError", angleError);
    atTarget = table.get("AtTarget", atTarget);
  }

  public HoodInputsAutoLogged clone() {
    HoodInputsAutoLogged copy = new HoodInputsAutoLogged();
    copy.currentAngle = this.currentAngle;
    copy.targetAngle = this.targetAngle;
    copy.angleError = this.angleError;
    copy.atTarget = this.atTarget;
    return copy;
  }
}
