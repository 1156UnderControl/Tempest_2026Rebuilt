
package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

    @AutoLog
  public class SpindexerInputs {
    public Distance spindexerPosition = Meters.of(0);
    public LinearVelocity spindexerVelocity = MetersPerSecond.of(0);
    public Voltage spindexerAppliedVolts = Volts.of(0);
    public Current spindexerCurrent = Amps.of(0);
  }
