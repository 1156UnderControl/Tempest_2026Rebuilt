package frc.robot.subsystems.hopper.spindexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

  private final SparkFlex leadSparkFlex;

  private final SparkFlex followSparkFlex;

  private SparkFlexConfig followerConfig;

  private final SmartMotorControllerConfig smartSpindexerMotorConfig;

  private final SmartMotorController spindexerMotor;

  private final FlyWheelConfig spindexerMechanismConfig;

  private final FlyWheel spindexer;

  private VelocityTorqueCurrentFOC torqueCurrentRequest = new VelocityTorqueCurrentFOC(RPM.of(0));

  private SpindexerSubsystem() {
    this.leadSparkFlex = new SparkFlex(SpindexerConstants.LEADER_MOTOR_ID, MotorType.kBrushless);
    this.followSparkFlex = new SparkFlex(SpindexerConstants.FOLLOW_MOTOR_ID, MotorType.kBrushless);

    this.followerConfig = new SparkFlexConfig();
    this.followerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40);
    this.followSparkFlex.configure(
        followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    this.smartSpindexerMotorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.OPEN_LOOP)
            .withFollowers(Pair.of(followSparkFlex, true))
            .withClosedLoopController(
                10, 0, 0, SpindexerConstants.MAX_VELOCITY, SpindexerConstants.MAX_ACCELERATION)
            .withSimClosedLoopController(
                0.03, 0, 0, SpindexerConstants.MAX_VELOCITY, SpindexerConstants.MAX_ACCELERATION)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))
            .withIdleMode(MotorMode.COAST)
            .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.9, 0))
            .withTelemetry(TelemetryVerbosity.MID)
            .withMechanismCircumference(Meters.of(0.478))
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(true);

    this.torqueCurrentRequest.withFeedForward(SpindexerConstants.FEEDFORWARD_TORQUE_CURRENT_A);

    this.spindexerMotor =
        new SparkWrapper(leadSparkFlex, DCMotor.getNeoVortex(2), smartSpindexerMotorConfig);

    this.spindexerMechanismConfig =
        new FlyWheelConfig(spindexerMotor)
            .withDiameter(Inches.of(6))
            .withMass(Pounds.of(0.58))
            .withMOI(Inches.of(6), Kilograms.of(0.58));

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
    spindexerInputs.spindexerLeadCurrent = spindexerMotor.getStatorCurrent();
    spindexerInputs.spindexerFollowerCurrent = Amps.of(followSparkFlex.getOutputCurrent());
    spindexerInputs.spindexerPosition = spindexerMotor.getMechanismPosition();
    spindexerInputs.spindexerVelocity = spindexerMotor.getMechanismVelocity().in(RPM);
    spindexerInputs.spindexerSetpoint = this.setpoint;
  }

  public Command shoot() {
    return spindexer.setVoltage(Volts.of(12));
  }

  public Command expell() {
    return spindexer.setVoltage(Volts.of(0));
  }

  public Command stop() {
    return spindexer.setVoltage(Volts.of(0));
  }

  public Command setDutyCycle(double dutycycle) {
    return spindexer.set(dutycycle);
  }

  public Trigger isAtVelocity() {
    return spindexer.isNear(this.setpoint, SpindexerConstants.DEADBAND_IS_AT);
  }
}
