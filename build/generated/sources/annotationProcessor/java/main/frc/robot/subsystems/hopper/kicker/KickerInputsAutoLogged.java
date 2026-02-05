package frc.robot.subsystems.hopper.kicker;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class KickerInputsAutoLogged extends KickerInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("KickerPosition", kickerPosition);
    table.put("KickerVelocity", kickerVelocity);
    table.put("KickerAppliedVolts", kickerAppliedVolts);
    table.put("KickerCurrent", kickerCurrent);
    table.put("KickerSetpoint", kickerSetpoint);
  }

  @Override
  public void fromLog(LogTable table) {
    kickerPosition = table.get("KickerPosition", kickerPosition);
    kickerVelocity = table.get("KickerVelocity", kickerVelocity);
    kickerAppliedVolts = table.get("KickerAppliedVolts", kickerAppliedVolts);
    kickerCurrent = table.get("KickerCurrent", kickerCurrent);
    kickerSetpoint = table.get("KickerSetpoint", kickerSetpoint);
  }

  public KickerInputsAutoLogged clone() {
    KickerInputsAutoLogged copy = new KickerInputsAutoLogged();
    copy.kickerPosition = this.kickerPosition;
    copy.kickerVelocity = this.kickerVelocity;
    copy.kickerAppliedVolts = this.kickerAppliedVolts;
    copy.kickerCurrent = this.kickerCurrent;
    copy.kickerSetpoint = this.kickerSetpoint;
    return copy;
  }
}
