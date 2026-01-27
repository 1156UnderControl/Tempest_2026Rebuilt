package frc.robot.constants.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class IntakeConstants {
  public static final int ID_intakeLeaderRollerMotor = 2;
  public static final int ID_intakeFollowerRollerMotor = 3;
  public static final int ID_intakeExtenderMotor = 4;

  public class motor_values {
    public static final double INTAKE_WHEELS_DUTY_CYCLE = 0;
    public static final double INTAKE_WHEELS_STOP = 0;
  }

  public class limits {
    public static final Distance MAX_EXTENSION = Meters.of(0.5);
    public static final Distance MIN_EXTENSION = Meters.of(0);
  }

  public class roller {
    public static final Distance MECHANISM_CIRCUNFERENCE = Inches.of(4);


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
  }

  public class extensor {

    public static final Distance MECHANISM_CIRCUNFERENCE = Meters.of(Math.PI * 0.0305);

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
      public static final double INTAKE_EXTENDED_POSITION = 0;
      public static final double INTAKE_HOMED_POSITION = 0;
    }
  }
}
