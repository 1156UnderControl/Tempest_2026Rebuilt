package frc.java_is_undercontrol.motors;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class MotorIOInputsAutoLogged extends MotorIO.MotorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("AppliedOutput", appliedOutput);
    table.put("Current", current);
    table.put("Position", position);
    table.put("Velocity", velocity);
    table.put("Temperature", temperature);
    table.put("Faults", faults);
    table.put("TargetPosition", targetPosition);
    table.put("TargetSpeed", targetSpeed);
    table.put("IsInverted", isInverted);
  }

  @Override
  public void fromLog(LogTable table) {
    appliedOutput = table.get("AppliedOutput", appliedOutput);
    current = table.get("Current", current);
    position = table.get("Position", position);
    velocity = table.get("Velocity", velocity);
    temperature = table.get("Temperature", temperature);
    faults = table.get("Faults", faults);
    targetPosition = table.get("TargetPosition", targetPosition);
    targetSpeed = table.get("TargetSpeed", targetSpeed);
    isInverted = table.get("IsInverted", isInverted);
  }

  public MotorIOInputsAutoLogged clone() {
    MotorIOInputsAutoLogged copy = new MotorIOInputsAutoLogged();
    copy.appliedOutput = this.appliedOutput;
    copy.current = this.current;
    copy.position = this.position;
    copy.velocity = this.velocity;
    copy.temperature = this.temperature;
    copy.faults = this.faults;
    copy.targetPosition = this.targetPosition;
    copy.targetSpeed = this.targetSpeed;
    copy.isInverted = this.isInverted;
    return copy;
  }
}
