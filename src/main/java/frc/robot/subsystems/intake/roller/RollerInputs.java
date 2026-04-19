package frc.robot.subsystems.intake.roller;

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
public class RollerInputs {
  Voltage rollerVoltage = Volts.of(0);
  Current rollerLeadCurrent = Amps.of(0);
  Current rollerFollowerCurrent = Amps.of(0);
  Angle rollerPosition = Degrees.of(0);
  AngularVelocity rollerVelocity = RPM.of(0);
  AngularVelocity rollerSetpoint = RPM.of(0);
}
