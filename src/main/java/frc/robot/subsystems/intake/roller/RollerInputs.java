package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class RollerInputs {
  public Voltage rollerRightAppliedVolts = Volts.of(0);
  public Voltage rollerLeftAppliedVolts = Volts.of(0);
  public Current rollerRightCurrent = Amps.of(0);
  public Current rollerLeftCurrent = Amps.of(0);
  public AngularVelocity rollerRightVelocity = RPM.of(0);
  public AngularVelocity rollerLeftVelocity = RPM.of(0);
}
