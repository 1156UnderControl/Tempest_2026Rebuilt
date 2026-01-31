package frc.robot.subsystems.intake.extensor;

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
public class ExtensorInputs {
  Voltage extensorVoltage = Volts.of(0);
  Current extensorCurrent = Amps.of(0);
  Distance extensorPosition = Meters.of(0);
  Distance extensorSetpoint = Meters.of(0);
  LinearVelocity extensorVelocity = MetersPerSecond.of(0);
}
