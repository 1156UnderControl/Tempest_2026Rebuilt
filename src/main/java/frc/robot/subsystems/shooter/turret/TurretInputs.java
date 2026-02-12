package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class TurretInputs {
  public double currentAngle = 0;
  public double targetAngle = 0;
  public Current current = Amps.of(0);
  public Voltage voltage = Volts.of(0);
  public boolean atTarget = false;
  public double distanceToTarget = Double.NaN;
  public double absoluteAngle;
  public Pose2d turret;
  public double encoder1;
  public double encoder2;
}
