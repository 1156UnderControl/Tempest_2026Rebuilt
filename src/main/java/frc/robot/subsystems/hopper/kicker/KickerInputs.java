package frc.robot.subsystems.hopper.kicker;

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
public class KickerInputs {
  public Angle kickerPosition = Degrees.of(0);
  public AngularVelocity kickerVelocity = RPM.of(0);
  public Voltage kickerAppliedVolts = Volts.of(0);
  public Current kickerCurrent = Amps.of(0);
  public AngularVelocity kickerSetpoint = RPM.of(0);
}
