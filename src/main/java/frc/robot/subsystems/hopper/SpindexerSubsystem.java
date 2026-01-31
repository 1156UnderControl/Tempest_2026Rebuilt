package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.hopper.SpindexerConstants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class SpindexerSubsystem extends SubsystemBase {
  private static SpindexerSubsystem instance;

  public static SpindexerSubsystem getInstance() {
    if (instance == null) {
      instance = new SpindexerSubsystem();
    }
    return instance;
  }

  private final SpindexerInputsAutoLogged spindexerInputs = new SpindexerInputsAutoLogged();

  private final SparkFlex spindexerFlex =
      new SparkFlex(SpindexerConstants.MOTOR_ID, SpindexerConstants.MOTOR_TYPE);

  private final SmartMotorControllerConfig smartSpindexerMotorConfig =
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
          .withMotorInverted(SpindexerConstants.Position.MOTOR_INVERTED)
          .withTelemetry(SpindexerConstants.MOTOR_NAME, SpindexerConstants.TELEMETRY);

  private final SmartMotorController spindexerMotor =
      new SparkWrapper(spindexerFlex, SpindexerConstants.MOTOR, smartSpindexerMotorConfig);

  private final FlyWheelConfig spindexerMechanismConfig =
      new FlyWheelConfig(spindexerMotor)
          .withDiameter(SpindexerConstants.Wheel.DIAMETER)
          .withMass(SpindexerConstants.Wheel.MASS)
          .withMOI(SpindexerConstants.Wheel.MOI)
          .withTelemetry(SpindexerConstants.MOTOR_NAME, SpindexerConstants.TELEMETRY);

  private final FlyWheel spindexer = new FlyWheel(spindexerMechanismConfig);

  public SpindexerSubsystem() {}

  @Override
  public void periodic() {
    this.updateInputs();
  }

  @Override
  public void simulationPeriodic() {}

  public void updateInputs() {
    spindexerInputs.spindexerAppliedVolts = spindexerMotor.getVoltage();
    spindexerInputs.spindexerCurrent = spindexerMotor.getStatorCurrent();
    spindexerInputs.spindexerPosition = spindexerMotor.getMeasurementPosition();
    spindexerInputs.spindexerVelocity = spindexerMotor.getMeasurementVelocity();
  }

  public Command setDutyCycle(double dutycycle){
    return spindexer.set(dutycycle);
  }
}
