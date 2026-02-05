package frc.robot.subsystems.intake.extensor;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ExtensorInputsAutoLogged extends ExtensorInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ExtensorVoltage", extensorVoltage);
    table.put("ExtensorCurrent", extensorCurrent);
    table.put("ExtensorPosition", extensorPosition);
    table.put("ExtensorSetpoint", extensorSetpoint);
    table.put("ExtensorVelocity", extensorVelocity);
  }

  @Override
  public void fromLog(LogTable table) {
    extensorVoltage = table.get("ExtensorVoltage", extensorVoltage);
    extensorCurrent = table.get("ExtensorCurrent", extensorCurrent);
    extensorPosition = table.get("ExtensorPosition", extensorPosition);
    extensorSetpoint = table.get("ExtensorSetpoint", extensorSetpoint);
    extensorVelocity = table.get("ExtensorVelocity", extensorVelocity);
  }

  public ExtensorInputsAutoLogged clone() {
    ExtensorInputsAutoLogged copy = new ExtensorInputsAutoLogged();
    copy.extensorVoltage = this.extensorVoltage;
    copy.extensorCurrent = this.extensorCurrent;
    copy.extensorPosition = this.extensorPosition;
    copy.extensorSetpoint = this.extensorSetpoint;
    copy.extensorVelocity = this.extensorVelocity;
    return copy;
  }
}
