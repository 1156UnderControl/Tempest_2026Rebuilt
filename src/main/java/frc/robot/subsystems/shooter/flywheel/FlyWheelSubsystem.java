package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.shooter.FlyWheelsConstants;
import frc.robot.subsystems.shooter.ShooterCalculator;
import org.littletonrobotics.junction.Logger;

import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlyWheelSubsystem extends SubsystemBase {

  private final SmartMotorControllerConfig leadConfig;
  private final TalonFX leadMotor;
  private final SmartMotorController smartLeadMotor;
  private final FlyWheelConfig leadMechanismConfig;
  private final FlyWheel leadMechanism;

  private final SmartMotorControllerConfig followerConfig;
  private final TalonFX followerMotor;
  private final SmartMotorController smartFollowerMotor;
  private final FlyWheelConfig followerMechanismConfig;
  private final FlyWheel followerMechanism;

  private final FlyWheelInputsAutoLogged inputs;

  private static FlyWheelSubsystem instance;

  private FlyWheelSubsystem() {
    this.leadConfig =
        new SmartMotorControllerConfig(this)
            .withIdleMode(FlyWheelsConstants.MOTOR_MODE)
            .withGearing(FlyWheelsConstants.MOTOR_GEARING)
            .withMotorInverted(FlyWheelsConstants.Position.MOTOR_INVERTED_LEAD)
            .withTelemetry(FlyWheelsConstants.MOTOR_LEAD_NAME, FlyWheelsConstants.TELEMETRY)
            .withStatorCurrentLimit(FlyWheelsConstants.STATOR_CURRENT)
            .withClosedLoopRampRate(FlyWheelsConstants.CLOSED_RAMP_RATE)
            .withClosedLoopController(new ExponentialProfilePIDController(new PIDController(
                FlyWheelsConstants.ClosedLoopController.kP,
                FlyWheelsConstants.ClosedLoopController.kI,
                FlyWheelsConstants.ClosedLoopController.kD),
                ExponentialProfilePIDController.createFlywheelConstraints(FlyWheelsConstants.MAX_VOLTS, FlyWheelsConstants.FLYWHEEL_MOTOR, FlyWheelsConstants.FLYWHEEL_MASS, FlyWheelsConstants.FLYWHEEL_SIZE, FlyWheelsConstants.MOTOR_GEARING)))
            .withSimClosedLoopController(new ExponentialProfilePIDController(new PIDController(
                FlyWheelsConstants.ClosedLoopController.kP,
                FlyWheelsConstants.ClosedLoopController.kI,
                FlyWheelsConstants.ClosedLoopController.kD),
                ExponentialProfilePIDController.createFlywheelConstraints(FlyWheelsConstants.MAX_VOLTS, FlyWheelsConstants.FLYWHEEL_MOTOR, FlyWheelsConstants.FLYWHEEL_MASS, FlyWheelsConstants.FLYWHEEL_SIZE, FlyWheelsConstants.MOTOR_GEARING)))
            .withFeedforward(
                new SimpleMotorFeedforward(
                    FlyWheelsConstants.FeedForward.kS,
                    FlyWheelsConstants.FeedForward.kV,
                    FlyWheelsConstants.FeedForward.kA))
            .withSimFeedforward(
                new SimpleMotorFeedforward(
                    FlyWheelsConstants.FeedForward.SIM.kS,
                    FlyWheelsConstants.FeedForward.SIM.kV,
                    FlyWheelsConstants.FeedForward.SIM.kA))
            .withControlMode(FlyWheelsConstants.CONTROL_MODE);

    this.leadMotor = new TalonFX(FlyWheelsConstants.FLYWHEEL_MOTOR_LEAD_ID);

    this.smartLeadMotor =
        new TalonFXWrapper(this.leadMotor, FlyWheelsConstants.FLYWHEEL_MOTOR, this.leadConfig);

    this.leadMechanismConfig =
        new FlyWheelConfig(smartLeadMotor)
            .withMOI(FlyWheelsConstants.FLYWHEEL_SIZE, FlyWheelsConstants.FLYWHEEL_MASS);

    this.leadMechanism = new FlyWheel(leadMechanismConfig);

    this.followerConfig =
        new SmartMotorControllerConfig(this)
            .withIdleMode(FlyWheelsConstants.MOTOR_MODE)
            .withGearing(FlyWheelsConstants.MOTOR_GEARING)
            .withMotorInverted(FlyWheelsConstants.Position.MOTOR_INVERTED_FOLLOWER)
            .withTelemetry(FlyWheelsConstants.MOTOR_FOLLOWER_NAME, FlyWheelsConstants.TELEMETRY)
            .withStatorCurrentLimit(FlyWheelsConstants.STATOR_CURRENT)
            .withClosedLoopRampRate(FlyWheelsConstants.CLOSED_RAMP_RATE)
            .withClosedLoopController(new ExponentialProfilePIDController(new PIDController(
                FlyWheelsConstants.ClosedLoopController.kP,
                FlyWheelsConstants.ClosedLoopController.kI,
                FlyWheelsConstants.ClosedLoopController.kD),
                ExponentialProfilePIDController.createFlywheelConstraints(FlyWheelsConstants.MAX_VOLTS, FlyWheelsConstants.FLYWHEEL_MOTOR, FlyWheelsConstants.FLYWHEEL_MASS, FlyWheelsConstants.FLYWHEEL_SIZE, FlyWheelsConstants.MOTOR_GEARING)))
            .withSimClosedLoopController(new ExponentialProfilePIDController(new PIDController(
                FlyWheelsConstants.ClosedLoopController.kP,
                FlyWheelsConstants.ClosedLoopController.kI,
                FlyWheelsConstants.ClosedLoopController.kD),
                ExponentialProfilePIDController.createFlywheelConstraints(FlyWheelsConstants.MAX_VOLTS, FlyWheelsConstants.FLYWHEEL_MOTOR, FlyWheelsConstants.FLYWHEEL_MASS, FlyWheelsConstants.FLYWHEEL_SIZE, FlyWheelsConstants.MOTOR_GEARING)))
            .withFeedforward(
                new SimpleMotorFeedforward(
                    FlyWheelsConstants.FeedForward.kS,
                    FlyWheelsConstants.FeedForward.kV,
                    FlyWheelsConstants.FeedForward.kA))
            .withSimFeedforward(
                new SimpleMotorFeedforward(
                    FlyWheelsConstants.FeedForward.SIM.kS,
                    FlyWheelsConstants.FeedForward.SIM.kV,
                    FlyWheelsConstants.FeedForward.SIM.kA))
            .withControlMode(FlyWheelsConstants.CONTROL_MODE);

    this.followerMotor =
        new TalonFX(FlyWheelsConstants.FLYWHEEL_MOTOR_FOLLOWER_ID, CANBus.roboRIO());

    this.smartFollowerMotor =
        new TalonFXWrapper(
            this.followerMotor, FlyWheelsConstants.FLYWHEEL_MOTOR, this.followerConfig);

    this.followerMechanismConfig =
        new FlyWheelConfig(smartFollowerMotor)
            .withMOI(FlyWheelsConstants.FLYWHEEL_SIZE, FlyWheelsConstants.FLYWHEEL_MASS);

    this.followerMechanism = new FlyWheel(followerMechanismConfig);

    this.inputs = new FlyWheelInputsAutoLogged();
  }

  public static FlyWheelSubsystem getInstance() {
    if (instance == null) {
      instance = new FlyWheelSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    this.smartFollowerMotor.setDutyCycle(smartLeadMotor.getDutyCycle());
    this.leadMechanism.updateTelemetry();
    this.followerMechanism.updateTelemetry();
    this.updateInputs();
  }

  @Override
  public void simulationPeriodic() {
    this.leadMechanism.simIterate();
    this.followerMechanism.simIterate();
  }

  private void updateInputs() {
    this.inputs.currentVelocity = getCurrentVelocity();
    this.inputs.targetVelocity = ShooterCalculator.getFlyWheelsTargetVelocityValue();
    this.inputs.atTarget = isAtTargetVelocity(ShooterCalculator.getFlyWheelsTargetVelocityValue());

    Logger.processInputs("Subsystems/Shooter/FlyWheels/", inputs);
  }

  public void setVelocity(double dutyCycle) {
    this.smartLeadMotor.setDutyCycle(dutyCycle);
  }

  public double getCurrentVelocity() {
    return this.smartLeadMotor.getDutyCycle();
  }

  public boolean isAtTargetVelocity(double targetDutyCycle) {
    return this.smartLeadMotor.getDutyCycle()
            <= FlyWheelsConstants.Velocity.VELOCITY_ERROR_ALLOWED + targetDutyCycle
        && this.smartLeadMotor.getDutyCycle()
            >= -FlyWheelsConstants.Velocity.VELOCITY_ERROR_ALLOWED + targetDutyCycle;
  }
}
