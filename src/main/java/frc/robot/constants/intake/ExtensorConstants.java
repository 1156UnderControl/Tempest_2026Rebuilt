package frc.robot.constants.intake;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;

public class ExtensorConstants {
  public static final int ID_MOTOR = 4;

  public class limits {
    public static final Distance MAX_EXTENSION = Meters.of(0.25);
    public static final Distance MIN_EXTENSION = Meters.of(0);
  }

  public static final Voltage maxVolts = Volts.of(10);
  public static final DCMotor motor = DCMotor.getNeoVortex(1);
  public static final Mass mass = Kilograms.of(5);
  public static final Distance drumRadius = Meters.of(0.0305 / 2.0);
  public static final MechanismGearing gearing =
      new MechanismGearing(GearBox.fromReductionStages(4));

  public static final Distance MECHANISM_CIRCUNFERENCE =
      Meters.of(Math.PI * 2 * drumRadius.in(Meters));

  public static final MechanismPositionConfig robotToMechanism =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(1.5))
          .withMaxRobotLength(Meters.of(0.75))
          .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)))
          .withMovementPlane(Plane.XY);

  public static final Distance DEADBAND_IS_AT = Meters.of(0.01);

  public class setpoints {
    public static final Distance INTAKE_EXTENDED_POSITION = limits.MAX_EXTENSION;
    public static final Distance INTAKE_HOMED_POSITION = limits.MIN_EXTENSION;
  }
}
