package frc.robot.subsystems.hopper.indexer;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.hopper.SpindexerConstants;
import frc.robot.subsystems.hopper.SpindexerInputsAutoLogged;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
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

    spindexerMotor = new SparkFlex(SpindexerConstants.MOTOR_ID, SpindexerConstants.MOTOR_TYPE);

    this.smartSpindexerMotorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(SpindexerConstants.CONTROL_MODE)
          .withClosedLoopController(
              SpindexerConstants.ClosedLoopController.kP,
              SpindexerConstants.ClosedLoopController.kI,
              SpindexerConstants.ClosedLoopController.kD,
              SpindexerConstants.Velocity.MAX_VELOCITY,
              SpindexerConstants.Velocity.ACCELERATION)
          .withGearing(SpindexerConstants.MOTOR_GEARING)
          .withIdleMode(SpindexerConstants.MOTOR_MODE)
          .withMechanismCircumference(Meters.of(0.47))
          .withMotorInverted(SpindexerConstants.Position.MOTOR_INVERTED)
          .withTelemetry(SpindexerConstants.MOTOR_NAME, SpindexerConstants.TELEMETRY);

    this.smartSpindexerMotor = new SparkWrapper(this.spindexerMotor, SpindexerConstants.MOTOR, this.smartSpindexerMotorConfig);
    
    this.spindexerMechanismConfig =
      new FlyWheelConfig(this.smartSpindexerMotor)
          .withDiameter(SpindexerConstants.Wheel.DIAMETER)
          .withMass(SpindexerConstants.Wheel.MASS)
          .withMOI(SpindexerConstants.Wheel.MOI)
          .withTelemetry(SpindexerConstants.MOTOR_NAME, SpindexerConstants.TELEMETRY);

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
    spindexerInputs.spindexerTargetVelocity = this.smartSpindexerMotor.getMechanismSetpointVelocity().isEmpty() ? RotationsPerSecond.of(0) : this.smartSpindexerMotor.getMechanismSetpointVelocity().get();
  }

  public Command setVelocity(AngularVelocity velocity) {
    return spindexerMechanism.setSpeed(velocity);
  }
}
