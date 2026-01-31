package frc.robot.constants.intake;

import static edu.wpi.first.units.Units.Inches;
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

public class IntakeConstants {
  public static final int ID_intakeLeaderRollerMotor = 2;
  public static final int ID_intakeFollowerRollerMotor = 3;
  public static final int ID_intakeExtenderMotor = 4;

  public class limits {
    public static final Distance MAX_EXTENSION = Meters.of(0.25);
    public static final Distance MIN_EXTENSION = Meters.of(0);
  }

  public class roller {
    public static final Distance MECHANISM_CIRCUNFERENCE = Inches.of(4);

    public static final MechanismPositionConfig robotToMechanism =
        new MechanismPositionConfig()
            .withMaxRobotHeight(Meters.of(1.5))
            .withMaxRobotLength(Meters.of(0.75))
            .withRelativePosition(
                new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)));

    public class PID {
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
    }

    public class feedforward {
      public static final double kA = 0;
      public static final double kV = 0;
      public static final double kS = 0;
    }

    public class setpoints {
      public static final double INTAKE_DUTY_CYCLE = 0.8;
      public static final double REVERSE_DUTY_CYCLE = -0.8;
      public static final double STOP_DUTY_CYCLE = 0;
    }
  }

  public class extensor {
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

    public static final Distance DEADBAND_IS_AT = Meters.of(0.05);

    public class PID {
      public static final double kP = 4.5;
      public static final double kI = 0;
      public static final double kD = 0;
    }

    public class feedforward {
      public static final double kA = 0;
      public static final double kV = 0;
      public static final double kS = 0;
    }

    public class setpoints {
      public static final Distance INTAKE_EXTENDED_POSITION = limits.MAX_EXTENSION;
      public static final Distance INTAKE_HOMED_POSITION = limits.MIN_EXTENSION;
    }
  }
}
