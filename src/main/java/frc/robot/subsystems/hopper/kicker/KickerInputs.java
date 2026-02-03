package frc.robot.subsystems.hopper.kicker;

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
public class KickerInputs {
  public Distance kickerPosition = Meters.of(0);
  public AngularVelocity kickerVelocity = RotationsPerSecond.of(0);
  public AngularVelocity kickerTargetVelocity = RotationsPerSecond.of(0);
  public Voltage kickerAppliedVolts = Volts.of(0);
  public Current kickerCurrent = Amps.of(0);
}
