package frc.robot.subsystems.hopper.indexer;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.hopper.SpindexerConstants;
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

  private final SpindexerInputsAutoLogged spindexerInputs;
  private final SmartMotorControllerConfig smartSpindexerMotorConfig;
  private final SparkFlex spindexerMotor;
  private final SmartMotorController smartSpindexerMotor;
  private final FlyWheelConfig spindexerMechanismConfig;
  private final FlyWheel spindexerMechanism;

  public SpindexerSubsystem() {
    this.spindexerInputs = new SpindexerInputsAutoLogged();

    spindexerMotor = new SparkFlex(SpindexerConstants.MOTOR_ID, MotorType.kBrushless);

    this.smartSpindexerMotorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                SpindexerConstants.ClosedLoopController.kP,
                SpindexerConstants.ClosedLoopController.kI,
                SpindexerConstants.ClosedLoopController.kD,
                SpindexerConstants.Velocity.MAX_VELOCITY,
                SpindexerConstants.Velocity.ACCELERATION)
            .withGearing(SpindexerConstants.MOTOR_GEARING)
            .withIdleMode(MotorMode.BRAKE)
            .withMechanismCircumference(Meters.of(0.47))
            .withMotorInverted(false)
            .withTelemetry("Spindexer Motor", TelemetryVerbosity.HIGH);

    this.smartSpindexerMotor =
        new SparkWrapper(
            this.spindexerMotor, DCMotor.getNeoVortex(1), this.smartSpindexerMotorConfig);

    this.spindexerMechanismConfig =
        new FlyWheelConfig(this.smartSpindexerMotor)
            .withDiameter(Inches.of(4))
            .withMass(Kilograms.of(0.45))
            .withMOI(KilogramSquareMeters.of(0.003))
            .withTelemetry("Spindexer Mechanism", TelemetryVerbosity.HIGH);

    this.spindexerMechanism = new FlyWheel(spindexerMechanismConfig);
  }

  public static SpindexerSubsystem getInstance() {
    if (instance == null) {
      instance = new SpindexerSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    this.spindexerMechanism.updateTelemetry();
    this.updateInputs();
  }

  @Override
  public void simulationPeriodic() {
    this.spindexerMechanism.simIterate();
  }

  public void updateInputs() {
    spindexerInputs.spindexerAppliedVolts = this.smartSpindexerMotor.getVoltage();
    spindexerInputs.spindexerCurrent = this.smartSpindexerMotor.getStatorCurrent();
    spindexerInputs.spindexerPosition = this.smartSpindexerMotor.getMeasurementPosition();
    spindexerInputs.spindexerVelocity = this.smartSpindexerMotor.getMechanismVelocity();
    spindexerInputs.spindexerTargetVelocity =
        this.smartSpindexerMotor.getMechanismSetpointVelocity().isEmpty()
            ? RotationsPerSecond.of(0)
            : this.smartSpindexerMotor.getMechanismSetpointVelocity().get();
  }

  public Command setVelocity(AngularVelocity velocity) {
    return spindexerMechanism.setSpeed(velocity);
  }
}
