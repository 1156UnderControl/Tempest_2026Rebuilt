package frc.robot.subsystems.hopper.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class SpindexerInputs {
  public Distance spindexerPosition = Meters.of(0);
  public AngularVelocity spindexerVelocity = RotationsPerSecond.of(0);
  public AngularVelocity spindexerTargetVelocity = RotationsPerSecond.of(0);
  public Voltage spindexerAppliedVolts = Volts.of(0);
  public Current spindexerCurrent = Amps.of(0);
}
