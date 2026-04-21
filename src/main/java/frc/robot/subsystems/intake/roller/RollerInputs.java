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
  public Voltage rollerLeadAppliedVolts = Volts.of(0);
  public Voltage rollerFollowerAppliedVolts = Volts.of(0);
  public Current rollerLeadCurrent = Amps.of(0);
  public Current rollerFollowerCurrent = Amps.of(0);
  public AngularVelocity rollerLeadVelocity = RPM.of(0);
  public AngularVelocity rollerFollowerVelocity = RPM.of(0);
}
