package frc.robot.subsystems.intake.roller;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class RollerInputsAutoLogged extends RollerInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("RollerVoltage", rollerVoltage);
    table.put("RollerCurrent", rollerCurrent);
    table.put("RollerPosition", rollerPosition);
    table.put("RollerVelocity", rollerVelocity);
    table.put("RollerSetpoint", rollerSetpoint);
  }

  @Override
  public void fromLog(LogTable table) {
    rollerVoltage = table.get("RollerVoltage", rollerVoltage);
    rollerCurrent = table.get("RollerCurrent", rollerCurrent);
    rollerPosition = table.get("RollerPosition", rollerPosition);
    rollerVelocity = table.get("RollerVelocity", rollerVelocity);
    rollerSetpoint = table.get("RollerSetpoint", rollerSetpoint);
  }

  public RollerInputsAutoLogged clone() {
    RollerInputsAutoLogged copy = new RollerInputsAutoLogged();
    copy.rollerVoltage = this.rollerVoltage;
    copy.rollerCurrent = this.rollerCurrent;
    copy.rollerPosition = this.rollerPosition;
    copy.rollerVelocity = this.rollerVelocity;
    copy.rollerSetpoint = this.rollerSetpoint;
    return copy;
  }
}
