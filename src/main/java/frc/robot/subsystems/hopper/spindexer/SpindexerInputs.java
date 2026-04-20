package frc.robot.subsystems.hopper.spindexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class SpindexerInputs {
  public Voltage spindexerLeadAppliedVolts = Volts.of(0);
  public Voltage spindexerFollowerAppliedVolts = Volts.of(0);
  public Current spindexerLeadCurrent = Amps.of(0);
  public Current spindexerFollowerCurrent = Amps.of(0);
  public AngularVelocity spindexerLeadVelocity = RPM.of(0);
  public AngularVelocity spindexerFollowerVelocity = RPM.of(0);
}
