package frc.java_is_undercontrol.sensors;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SensorIOInputsAutoLogged extends SensorIO.SensorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("Value", value);
    table.put("Inverted", inverted);
    table.put("TimestampSec", timestampSec);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    value = table.get("Value", value);
    inverted = table.get("Inverted", inverted);
    timestampSec = table.get("TimestampSec", timestampSec);
  }

  public SensorIOInputsAutoLogged clone() {
    SensorIOInputsAutoLogged copy = new SensorIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.value = this.value;
    copy.inverted = this.inverted;
    copy.timestampSec = this.timestampSec;
    return copy;
  }
}
