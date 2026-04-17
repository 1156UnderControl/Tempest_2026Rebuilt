package frc.robot.subsystems.hopper.spindexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class SpindexerInputs {
  public Angle spindexerPosition = Degrees.of(0);
  public double spindexerVelocity = 0;
  public Voltage spindexerAppliedVolts = Volts.of(0);
  public Current spindexerCurrent = Amps.of(0);
  public AngularVelocity spindexerSetpoint = RPM.of(0);
}
