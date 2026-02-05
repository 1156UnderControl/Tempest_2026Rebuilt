package frc.robot.subsystems.climber;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberInputsAutoLogged extends ClimberInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ClimberPosition", climberPosition);
    table.put("ClimberVelocity", climberVelocity);
    table.put("ClimberSetpoint", climberSetpoint);
    table.put("ClimberAppliedVolts", climberAppliedVolts);
    table.put("ClimberCurrent", climberCurrent);
  }

  @Override
  public void fromLog(LogTable table) {
    climberPosition = table.get("ClimberPosition", climberPosition);
    climberVelocity = table.get("ClimberVelocity", climberVelocity);
    climberSetpoint = table.get("ClimberSetpoint", climberSetpoint);
    climberAppliedVolts = table.get("ClimberAppliedVolts", climberAppliedVolts);
    climberCurrent = table.get("ClimberCurrent", climberCurrent);
  }

  public ClimberInputsAutoLogged clone() {
    ClimberInputsAutoLogged copy = new ClimberInputsAutoLogged();
    copy.climberPosition = this.climberPosition;
    copy.climberVelocity = this.climberVelocity;
    copy.climberSetpoint = this.climberSetpoint;
    copy.climberAppliedVolts = this.climberAppliedVolts;
    copy.climberCurrent = this.climberCurrent;
    return copy;
  }
}
