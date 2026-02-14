package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.shooter.HoodConstants;
import frc.robot.subsystems.shooter.ShooterCalculator;
import org.littletonrobotics.junction.Logger;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class HoodSubsystem extends SubsystemBase {

  private final ShooterCalculator calculator;

  private final SparkMax hoodMax;
  private final SmartMotorControllerConfig smartHoodMotorConfig;
  private final SmartMotorController hoodMotor;
  private final ArmConfig hoodMechanismConfig;
  private final Arm hood;

  private final HoodInputsAutoLogged inputs;

  private static HoodSubsystem instance;

  private HoodSubsystem() {
    calculator = ShooterCalculator.getInstance();
    smartHoodMotorConfig =
        new SmartMotorControllerConfig(this)
            .withIdleMode(MotorMode.BRAKE)
            .withGearing(HoodConstants.MOTOR_GEARING)
            .withMotorInverted(false)
            .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
            .withSoftLimit(
                HoodConstants.Position.HOOD_MIN_ANGLE, HoodConstants.Position.HOOD_MAX_ANGLE)
            .withStatorCurrentLimit(Amps.of(60))
            .withClosedLoopRampRate(Milliseconds.of(100))
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
            .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withControlMode(ControlMode.CLOSED_LOOP);

    hoodMax = new SparkMax(8, MotorType.kBrushless);

    hoodMotor = new SparkWrapper(hoodMax, HoodConstants.HOOD_MOTOR, smartHoodMotorConfig);

    hoodMechanismConfig =
        new ArmConfig(hoodMotor)
            .withStartingPosition(Degrees.of(75))
            .withHardLimit(
                HoodConstants.Position.HOOD_MIN_ANGLE, HoodConstants.Position.HOOD_MAX_ANGLE)
            .withLength(HoodConstants.HOOD_SIZE)
            .withMass(HoodConstants.HOOD_MASS);

    hood = new Arm(hoodMechanismConfig);

    inputs = new HoodInputsAutoLogged();
  }

  public static HoodSubsystem getInstance() {
    if (instance == null) {
      instance = new HoodSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    hood.updateTelemetry();
    updateInputs();
  }

  @Override
  public void simulationPeriodic() {
    hood.simIterate();
  }

  private void updateInputs() {
    inputs.currentAngle = hood.getAngle().in(Degrees);
    inputs.targetAngle = calculator.getParameters().hoodAngle();
    inputs.atTarget = isAtTargetAngleShoot();

    Logger.processInputs("Subsystems/Shooter/Hood/", inputs);
  }

  public Command goToTargetAngle() {
    return hood.setAngle(() -> Degrees.of(calculator.getParameters(true).hoodAngle()));
  }

  public boolean isAtTargetAngleShoot() {
    return hood.getAngle()
        .isNear(
            Degrees.of(calculator.getLastParameters().hoodAngle()),
            HoodConstants.Position.ANGLE_ERROR_ALLOWED);
  }

  public double getMinAngleDegrees() {
    return HoodConstants.Position.HOOD_MIN_ANGLE.in(Degrees);
  }

  public double getMaxAngleDegrees() {
    return HoodConstants.Position.HOOD_MAX_ANGLE.in(Degrees);
  }
}
