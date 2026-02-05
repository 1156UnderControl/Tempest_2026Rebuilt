package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ClimberInputs {
  public Distance climberPosition = Meters.of(0);
  public LinearVelocity climberVelocity = MetersPerSecond.of(0);
  public Distance climberSetpoint = Meters.of(0);
  public Voltage climberAppliedVolts = Volts.of(0);
  public Current climberCurrent = Amps.of(0);
}
