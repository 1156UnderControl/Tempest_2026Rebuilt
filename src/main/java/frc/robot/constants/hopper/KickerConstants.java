package frc.robot.constants.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/** Constants describing the kicker wheel that feeds the shooter. */
public final class KickerConstants {
  public static final int MOTOR_ID = 0;
  public static final String MOTOR_NAME = "KickerMotor";
  public static final DCMotor MOTOR = DCMotor.getNeoVortex(1);
  public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
  public static final MotorMode MOTOR_MODE = MotorMode.BRAKE;
  public static final MechanismGearing MOTOR_GEARING =
      new MechanismGearing(GearBox.fromReductionStages(4));
  public static final ControlMode CONTROL_MODE = ControlMode.CLOSED_LOOP;
  public static final TelemetryVerbosity TELEMETRY = TelemetryVerbosity.HIGH;
  public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
  public static final Time CLOSED_LOOP_RAMP = Seconds.of(0.25);
  public static final Time OPEN_LOOP_RAMP = Seconds.of(0.25);
  public static final Voltage MAX_VOLTS = Volts.of(12);

  private KickerConstants() {}

  public static final class Wheel {
    public static final Distance DIAMETER = Inches.of(4);
    public static final Mass MASS = Kilograms.of(0.45);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.003);

    private Wheel() {}
  }

  public static final class Velocity {
    public static final AngularVelocity MAX_VELOCITY = RPM.of(3000);
    public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(150);

    private Velocity() {}
  }

  public static final class Position {
    public static final boolean MOTOR_INVERTED = false;

    private Position() {}
  }

  public static final class ClosedLoopController {
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    private ClosedLoopController() {}
  }

  public static final class FeedForward {
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    private FeedForward() {}
  }
}
