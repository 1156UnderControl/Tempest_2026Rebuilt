package frc.robot.subsystems.hopper.spindexer;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SpindexerInputsAutoLogged extends SpindexerInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("SpindexerPosition", spindexerPosition);
    table.put("SpindexerVelocity", spindexerVelocity);
    table.put("SpindexerAppliedVolts", spindexerAppliedVolts);
    table.put("SpindexerCurrent", spindexerCurrent);
    table.put("SpindexerSetpoint", spindexerSetpoint);
  }

  @Override
  public void fromLog(LogTable table) {
    spindexerPosition = table.get("SpindexerPosition", spindexerPosition);
    spindexerVelocity = table.get("SpindexerVelocity", spindexerVelocity);
    spindexerAppliedVolts = table.get("SpindexerAppliedVolts", spindexerAppliedVolts);
    spindexerCurrent = table.get("SpindexerCurrent", spindexerCurrent);
    spindexerSetpoint = table.get("SpindexerSetpoint", spindexerSetpoint);
  }

  public SpindexerInputsAutoLogged clone() {
    SpindexerInputsAutoLogged copy = new SpindexerInputsAutoLogged();
    copy.spindexerPosition = this.spindexerPosition;
    copy.spindexerVelocity = this.spindexerVelocity;
    copy.spindexerAppliedVolts = this.spindexerAppliedVolts;
    copy.spindexerCurrent = this.spindexerCurrent;
    copy.spindexerSetpoint = this.spindexerSetpoint;
    return copy;
  }
}
