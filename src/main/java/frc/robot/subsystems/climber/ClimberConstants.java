package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class ClimberConstants {
  public static final int CLIMBER_MOTOR_ID = 5;

  public static final MotorType MOTOR_TYPE = MotorType.kBrushless;

  public static final MotorMode MOTOR_MODE = MotorMode.BRAKE;

  public static final TelemetryVerbosity TELEMETRY = TelemetryVerbosity.HIGH;

  public static final DCMotor DC_MOTOR = DCMotor.getNeoVortex(1);

  public static final String CLIMBER_MECHANISM_NAME = "Climber";

  public static final Voltage MAX_VOLTS = Volts.of(12.0);

  public static final ControlMode CONTROL_MODE = ControlMode.CLOSED_LOOP;

  public static final Distance MECHANISM_CIRCUNFERENCE = Meters.of(2 * Math.PI * 0.015875);

  public static final MechanismGearing GEARING =
      new MechanismGearing(GearBox.fromReductionStages(3, 3, 4));

  public static final boolean INVERTED = true;

  public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);

  public static final Distance HEIGHT_ERROR_ALLOWED = Meters.of(0);

  public static final Distance CLIMBER_MIN_HEIGHT = Meters.of(0);

  public static final Distance CLIMBER_MAX_HEIGHT = Meters.of(0.17);

  public static final Time CLOSED_RAMP_RATE = Milliseconds.of(10);

  public static final Distance STARTING_HEIGHT = Meters.of(0);

  public static final Mass CLIMBER_MASS = Kilograms.of(2);

  public static class Setpoints {
    public static final Distance DEFAULT_HEIGHT = Meters.of(0);
    public static final Distance CLIMBED_HEIGHT = Meters.of(0);
  }

  public static class FeedBackController {
    public static final double kP = 500;
    public static final double kI = 0;
    public static final double kD = 0;

    public static class SIM {
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
    }
  }

  public static class FeedForwardController {
    public static final double kV = 0;
    public static final double kS = 0;
    public static final double kA = 0;

    public static class SIM {
      public static final double kV = 200;
      public static final double kS = 0;
      public static final double kA = 0;
    }
  }
}
