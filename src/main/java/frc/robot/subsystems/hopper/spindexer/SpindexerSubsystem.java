package frc.robot.subsystems.hopper.spindexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.hopper.SpindexerConstants;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class SpindexerSubsystem extends SubsystemBase {
  private static SpindexerSubsystem instance;

  public static SpindexerSubsystem getInstance() {
    if (instance == null) {
      instance = new SpindexerSubsystem();
    }
    return instance;
  }

  private AngularVelocity setpoint = RPM.of(0);

  private final SpindexerInputsAutoLogged spindexerInputs = new SpindexerInputsAutoLogged();

  private final SparkFlex spindexerFlex;

  private final SmartMotorControllerConfig smartSpindexerMotorConfig;

  private final SmartMotorController spindexerMotor;

  private final FlyWheelConfig spindexerMechanismConfig;

  private final FlyWheel spindexer;

  private SpindexerSubsystem() {
    this.spindexerFlex = new SparkFlex(SpindexerConstants.MOTOR_ID, MotorType.kBrushless);

    this.smartSpindexerMotorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                0.1, 0, 0, SpindexerConstants.MAX_VELOCITY, SpindexerConstants.MAX_ACCELERATION)
            .withSimClosedLoopController(
                0.03, 0, 0, SpindexerConstants.MAX_VELOCITY, SpindexerConstants.MAX_ACCELERATION)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(20)))
            .withIdleMode(MotorMode.BRAKE)
            .withFeedforward(new SimpleMotorFeedforward(0.11, 2.2, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.9, 0))
            .withMechanismCircumference(Meters.of(0.478))
            .withStatorCurrentLimit(Amps.of(60))
            .withMotorInverted(false)
            .withTelemetry("SpindexerMotor", TelemetryVerbosity.HIGH);

    this.spindexerMotor =
        new SparkWrapper(spindexerFlex, DCMotor.getNeoVortex(1), smartSpindexerMotorConfig);

    this.spindexerMechanismConfig =
        new FlyWheelConfig(spindexerMotor)
            .withDiameter(Inches.of(6))
            .withMass(Pounds.of(0.58))
            .withMOI(Inches.of(6), Kilograms.of(0.58))
            .withTelemetry("SpindexerMech", TelemetryVerbosity.HIGH);

    this.spindexer = new FlyWheel(spindexerMechanismConfig);
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Subsystems/Spindexer", spindexerInputs);
    spindexer.updateTelemetry();
    this.setpoint =
        spindexerMotor.getMechanismSetpointVelocity().isEmpty()
            ? RPM.of(0)
            : spindexerMotor.getMechanismSetpointVelocity().get();
  }

  @Override
  public void simulationPeriodic() {
    this.spindexer.simIterate();
  }

  public void updateInputs() {
    spindexerInputs.spindexerAppliedVolts = spindexerMotor.getVoltage();
    spindexerInputs.spindexerCurrent = spindexerMotor.getStatorCurrent();
    spindexerInputs.spindexerPosition = spindexerMotor.getMechanismPosition();
    spindexerInputs.spindexerVelocity = spindexerMotor.getMechanismVelocity();
    spindexerInputs.spindexerSetpoint = this.setpoint;
  }

  public Command shoot() {
    return spindexer.setSpeed(SpindexerConstants.setpoints.SHOOTING);
  }

  public Command expell() {
    return spindexer.setSpeed(SpindexerConstants.setpoints.EXPELLING);
  }

  public Command stop() {
    return spindexer.setSpeed(SpindexerConstants.setpoints.STOPPED);
  }

  public Command setDutyCycle(double dutycycle) {
    return spindexer.set(dutycycle);
  }

  public Trigger isAtVelocity() {
    return spindexer.isNear(this.setpoint, SpindexerConstants.DEADBAND_IS_AT);
  }
}
