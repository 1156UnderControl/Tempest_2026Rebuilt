package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.shooter.HoodConstants;
import frc.robot.subsystems.shooter.ShooterCalculator;
import org.littletonrobotics.junction.Logger;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class HoodSubsystem extends SubsystemBase {

  private final SmartMotorControllerConfig motorConfig;
  private final SparkMax motor;
  private final SmartMotorController smartMotor;
  private final ArmConfig mechanismConfig;
  private final Arm mechanism;

  private final HoodInputsAutoLogged inputs;

  private static HoodSubsystem instance;

  private HoodSubsystem() {
    this.motorConfig =
        new SmartMotorControllerConfig(this)
            .withIdleMode(HoodConstants.MOTOR_MODE)
            .withGearing(HoodConstants.MOTOR_GEARING)
            .withMotorInverted(HoodConstants.Position.MOTOR_INVERTED)
            .withTelemetry(HoodConstants.MOTOR_NAME, HoodConstants.TELEMETRY)
            .withSoftLimit(
                HoodConstants.Position.HOOD_MIN_ANGLE, HoodConstants.Position.HOOD_MAX_ANGLE)
            .withStatorCurrentLimit(HoodConstants.STATOR_CURRENT)
            .withClosedLoopRampRate(HoodConstants.CLOSED_RAMP_RATE)
            .withClosedLoopController(
                new ExponentialProfilePIDController(
                    new PIDController(
                        HoodConstants.ClosedLoopController.kP,
                        HoodConstants.ClosedLoopController.kI,
                        HoodConstants.ClosedLoopController.kD),
                    ExponentialProfilePIDController.createArmConstraints(
                        HoodConstants.MAX_VOLTS,
                        HoodConstants.HOOD_MOTOR,
                        HoodConstants.HOOD_MASS,
                        HoodConstants.HOOD_SIZE,
                        HoodConstants.MOTOR_GEARING)))
            .withSimClosedLoopController(
                new ExponentialProfilePIDController(
                    new PIDController(
                        HoodConstants.ClosedLoopController.kP,
                        HoodConstants.ClosedLoopController.kI,
                        HoodConstants.ClosedLoopController.kD),
                    ExponentialProfilePIDController.createArmConstraints(
                        HoodConstants.MAX_VOLTS,
                        HoodConstants.HOOD_MOTOR,
                        HoodConstants.HOOD_MASS,
                        HoodConstants.HOOD_SIZE,
                        HoodConstants.MOTOR_GEARING)))
            .withFeedforward(
                new SimpleMotorFeedforward(
                    HoodConstants.FeedForward.kS,
                    HoodConstants.FeedForward.kV,
                    HoodConstants.FeedForward.kA))
            .withSimFeedforward(
                new SimpleMotorFeedforward(
                    HoodConstants.FeedForward.SIM.kS,
                    HoodConstants.FeedForward.SIM.kV,
                    HoodConstants.FeedForward.SIM.kA))
            .withControlMode(HoodConstants.CONTROL_MODE);

    this.motor = new SparkMax(HoodConstants.HOOD_MOTOR_ID, HoodConstants.MOTOR_TYPE);

    this.smartMotor = new SparkWrapper(this.motor, HoodConstants.HOOD_MOTOR, this.motorConfig);

    this.mechanismConfig =
        new ArmConfig(smartMotor)
            .withStartingPosition(HoodConstants.Position.STARTING_POSITION)
            .withHardLimit(
                HoodConstants.Position.HOOD_MIN_ANGLE, HoodConstants.Position.HOOD_MAX_ANGLE)
            .withLength(HoodConstants.HOOD_SIZE)
            .withMass(HoodConstants.HOOD_MASS);

    this.mechanism = new Arm(mechanismConfig);

    this.inputs = new HoodInputsAutoLogged();
  }

  public static HoodSubsystem getInstance() {
    if (instance == null) {
      instance = new HoodSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    this.mechanism.updateTelemetry();
    this.updateInputs();
  }

  @Override
  public void simulationPeriodic() {
    this.mechanism.simIterate();
  }

  private void updateInputs() {
    this.inputs.currentAngle = getCurrentAngleDegrees();
    this.inputs.targetAngle = ShooterCalculator.getHoodTargetAngleValue();
    this.inputs.atTarget = isAtTargetAngle(ShooterCalculator.getHoodTargetAngleValue());

    Logger.processInputs("Subsystems/Shooter/Hood/", inputs);
  }

  public void setTargetAngle(double angleDegrees) {
    this.smartMotor.setPosition(Angle.ofBaseUnits(angleDegrees, Degrees));
  }

  public void setTargetAngle(Angle angle) {
    setTargetAngle(angle.in(Degrees));
  }

  public void stop() {
    this.smartMotor.setDutyCycle(0);
  }

  public double getCurrentAngleDegrees() {
    return this.smartMotor.getMechanismPosition().in(Degrees);
  }

  public Angle getCurrentAngle() {
    return this.smartMotor.getMechanismPosition();
  }

  public boolean isAtTargetAngle(double targetAngleDegrees) {
    return this.smartMotor.getMechanismPosition().in(Degrees)
            <= HoodConstants.Position.ANGLE_ERROR_ALLOWED + targetAngleDegrees
        && this.smartMotor.getMechanismPosition().in(Degrees)
            >= -HoodConstants.Position.ANGLE_ERROR_ALLOWED + targetAngleDegrees;
  }

  public double getMinAngleDegrees() {
    return HoodConstants.Position.HOOD_MIN_ANGLE.in(Degrees);
  }

  public double getMaxAngleDegrees() {
    return HoodConstants.Position.HOOD_MAX_ANGLE.in(Degrees);
  }
}
