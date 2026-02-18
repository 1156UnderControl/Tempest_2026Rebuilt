package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStates;
import frc.robot.subsystems.shooter.ShooterCalculator;
import org.littletonrobotics.junction.Logger;
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

  private double angleCalibration;

  private HoodSubsystem() {
    calculator = ShooterCalculator.getInstance();
    smartHoodMotorConfig =
        new SmartMotorControllerConfig(this)
            .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
            .withIdleMode(MotorMode.BRAKE)
            .withGearing(HoodConstants.MOTOR_GEARING)
            .withMotorInverted(false)
            .withSoftLimit(
                HoodConstants.Position.HOOD_MIN_ANGLE, HoodConstants.Position.HOOD_MAX_ANGLE)
            .withStatorCurrentLimit(Amps.of(60))
            .withClosedLoopRampRate(Milliseconds.of(100))
            .withClosedLoopController(
                HoodConstants.ClosedLoopController.kP,
                HoodConstants.ClosedLoopController.kI,
                HoodConstants.ClosedLoopController.kD)
            .withSimClosedLoopController(
                HoodConstants.ClosedLoopController.kP,
                HoodConstants.ClosedLoopController.kI,
                HoodConstants.ClosedLoopController.kD)
            .withFeedforward(new SimpleMotorFeedforward(0.135, 4, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(0.135, 4, 0))
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

    SmartDashboard.putNumber("Calibration/Hood", 41);
    angleCalibration = 41;
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
    this.angleCalibration = SmartDashboard.getNumber("Calibration/Hood", 41);
  }

  @Override
  public void simulationPeriodic() {
    hood.simIterate();
  }

  private void updateInputs() {
    inputs.currentAngle = hood.getAngle().in(Degrees);
    inputs.targetAngle = calculator.getParameters().hoodAngle();
    inputs.atTarget = isAtTargetAngle();
    inputs.angleError =
        Math.abs(hood.getAngle().in(Degrees) - calculator.getParameters().hoodAngle());

    Logger.processInputs("Subsystems/Shooter/Hood/", inputs);
  }

  public Command trackHub() {
    return hood.setAngle(() -> Degrees.of(calculator.getParameters(true).hoodAngle()));
  }

  public Command trackPass() {
    return hood.setAngle(() -> Degrees.of(calculator.getParameters(false).hoodAngle()));
  }

  public Command trackNonOdometry() {
    return hood.setAngle(() -> Degrees.of(73));
  }

  public boolean isAtTargetAngle() {
    return hood.getAngle()
        .isNear(
            this.hood.getMechanismSetpoint().orElseGet(()-> Degrees.of(75)),
            HoodConstants.Position.ANGLE_ERROR_ALLOWED);
  }

  public double getMinAngleDegrees() {
    return HoodConstants.Position.HOOD_MIN_ANGLE.in(Degrees);
  }

  public double getMaxAngleDegrees() {
    return HoodConstants.Position.HOOD_MAX_ANGLE.in(Degrees);
  }

  public Command goToTargetAngleCalibration() {
    return hood.setAngle(() -> Degrees.of(angleCalibration));
  }
}
