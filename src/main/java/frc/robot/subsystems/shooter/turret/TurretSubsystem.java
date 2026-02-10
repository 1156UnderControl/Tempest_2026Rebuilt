package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.shooter.TurretConstants;
import frc.robot.subsystems.shooter.ShooterCalculator;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class TurretSubsystem extends SubsystemBase {

  private final ShooterCalculator calculator;
  private final TurretInputsAutoLogged inputs;
  private static TurretSubsystem instance;

  private final SparkFlex turretFlex;
  private final SmartMotorControllerConfig smartTurretMotorConfig;
  private final SmartMotorController turretMotor;
  private final PivotConfig turretMechanismConfig;
  private final Pivot turret;

  private final SparkAbsoluteEncoder encoder1;
  private final SparkAbsoluteEncoder encoder2;

  private final EasyCRT easyCRT;

  private double absoluteAngle = 2;

  private TurretSubsystem(SparkAbsoluteEncoder kickerEncoder, SparkAbsoluteEncoder climberEncoder) {
    calculator = ShooterCalculator.getInstance();
    smartTurretMotorConfig =
        new SmartMotorControllerConfig(this)
            .withIdleMode(MotorMode.BRAKE)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(60.3428)))
            .withMotorInverted(false)
            .withTelemetry("TurretMotor", TurretConstants.TELEMETRY)
            .withSoftLimit(
                TurretConstants.Position.TURRET_MIN_ANGLE,
                TurretConstants.Position.TURRET_MAX_ANGLE)
            .withStatorCurrentLimit(Amps.of(40))
            .withClosedLoopRampRate(Seconds.of(0.5))
            .withClosedLoopController(
                new ExponentialProfilePIDController(
                    new PIDController(
                        TurretConstants.ClosedLoopController.kP,
                        TurretConstants.ClosedLoopController.kI,
                        TurretConstants.ClosedLoopController.kD),
                    ExponentialProfilePIDController.createConstraints(
                        TurretConstants.MAX_VOLTS,
                        TurretConstants.Velocity.MAX_VELOCITY,
                        TurretConstants.Velocity.ACCELERATION)))
            .withSimClosedLoopController(
                new ExponentialProfilePIDController(
                    new PIDController(
                        TurretConstants.ClosedLoopController.kP,
                        TurretConstants.ClosedLoopController.kI,
                        TurretConstants.ClosedLoopController.kD),
                    ExponentialProfilePIDController.createConstraints(
                        TurretConstants.MAX_VOLTS,
                        TurretConstants.Velocity.MAX_VELOCITY,
                        TurretConstants.Velocity.ACCELERATION)))
            .withFeedforward(new SimpleMotorFeedforward(0.5, 0.5, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withControlMode(ControlMode.CLOSED_LOOP);

    turretFlex = new SparkFlex(7, MotorType.kBrushless);

    turretMotor = new SparkWrapper(turretFlex, DCMotor.getNeoVortex(1), smartTurretMotorConfig);

    turretMechanismConfig =
        new PivotConfig(turretMotor)
            .withHardLimit(
                TurretConstants.Position.TURRET_MIN_ANGLE,
                TurretConstants.Position.TURRET_MAX_ANGLE)
            .withMOI(Meters.of(0.25), Kilograms.of(6))
            .withStartingPosition(TurretConstants.Position.TURRET_STARTING_ANGLE)
            .withTelemetry("Turret", TurretConstants.TELEMETRY);

    turret = new Pivot(turretMechanismConfig);

    encoder1 = kickerEncoder;
    // TODO: define second encoder correctly
    encoder2 = climberEncoder;

    EasyCRTConfig easyCRTConfig =
        new EasyCRTConfig(
                () -> Angle.ofBaseUnits(encoder1.getPosition(), Rotations),
                () -> Angle.ofBaseUnits(encoder2.getPosition(), Rotations))
            .withEncoderRatios(
                TurretConstants.TURRET_GEAR_TEETH / 14, TurretConstants.TURRET_GEAR_TEETH / 15)
            .withMechanismRange(
                TurretConstants.Position.TURRET_MIN_ANGLE,
                TurretConstants.Position.TURRET_MAX_ANGLE);

    this.easyCRT = new EasyCRT(easyCRTConfig);

    this.easyCRT
        .getAngleOptional()
        .ifPresent(
            mechAngle -> {
              turretMotor.setEncoderPosition(mechAngle);
              this.absoluteAngle = mechAngle.in(Degrees);
            });

    inputs = new TurretInputsAutoLogged();
  }

  public static TurretSubsystem getInstance(
      SparkAbsoluteEncoder kickerEncoder, SparkAbsoluteEncoder climberEncoder) {
    if (instance == null) {
      instance = new TurretSubsystem(kickerEncoder, climberEncoder);
    }
    return instance;
  }

  @Override
  public void periodic() {
    turret.updateTelemetry();

    updateInputs();
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }

  private void updateInputs() {
    inputs.currentAngle = turret.getAngle().in(Degrees);
    inputs.targetAngle = calculator.getParameters().turretAngle().getDegrees();
    inputs.atTarget = isAtTargetAngle();
    inputs.absoluteAngle = this.absoluteAngle;

    Logger.processInputs("Subsystems/Shooter/Turret/", inputs);
  }

  public Command trackHub() {
    return turret.setAngle(
        () ->
            Degrees.of(
                getReachableTurretAngle(calculator.getParameters().turretAngle().getDegrees())));
  }

  public Command trackPass() {
    return turret.setAngle(
        () ->
            Degrees.of(
                getReachableTurretAngle(
                    calculator.getParameters(false).turretAngle().getDegrees())));
  }

  public boolean isAtTargetAngle() {
    return turret
        .getAngle()
        .isNear(
            turret.getMechanismSetpoint().isEmpty() ? Degrees.of(0) : turret.getAngle(),
            TurretConstants.Position.ANGLE_ERROR_ALLOWED);
  }

  private double getReachableTurretAngle(double targetAngle) {
    if (targetAngle > TurretConstants.Position.TURRET_MAX_ANGLE.in(Degrees)) {
      return targetAngle - 360;
    } else if (targetAngle < TurretConstants.Position.TURRET_MIN_ANGLE.in(Degrees)) {
      return targetAngle + 360;
    } else {
      return targetAngle;
    }
  }
}
