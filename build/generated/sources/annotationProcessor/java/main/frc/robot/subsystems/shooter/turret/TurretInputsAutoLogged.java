package frc.robot.subsystems.shooter.turret;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class TurretInputsAutoLogged extends TurretInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("CurrentAngle", currentAngle);
    table.put("TargetAngle", targetAngle);
    table.put("AtTarget", atTarget);
    table.put("DistanceToTarget", distanceToTarget);
    table.put("AbsoluteAngle", absoluteAngle);
    table.put("Turret", turret);
  }

  @Override
  public void fromLog(LogTable table) {
    currentAngle = table.get("CurrentAngle", currentAngle);
    targetAngle = table.get("TargetAngle", targetAngle);
    atTarget = table.get("AtTarget", atTarget);
    distanceToTarget = table.get("DistanceToTarget", distanceToTarget);
    absoluteAngle = table.get("AbsoluteAngle", absoluteAngle);
    turret = table.get("Turret", turret);
  }

  public TurretInputsAutoLogged clone() {
    TurretInputsAutoLogged copy = new TurretInputsAutoLogged();
    copy.currentAngle = this.currentAngle;
    copy.targetAngle = this.targetAngle;
    copy.atTarget = this.atTarget;
    copy.distanceToTarget = this.distanceToTarget;
    copy.absoluteAngle = this.absoluteAngle;
    copy.turret = this.turret;
    return copy;
  }
}
