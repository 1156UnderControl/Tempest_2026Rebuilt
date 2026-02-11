package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStates;
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

  private double absoluteAngle;

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
                        TurretConstants.ClosedLoopController.SIM.kP,
                        TurretConstants.ClosedLoopController.SIM.kI,
                        TurretConstants.ClosedLoopController.SIM.kD),
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

    this.encoder1 = kickerEncoder;
    this.encoder2 = climberEncoder;

    this.absoluteAngle =
        determineTurretAngleDegrees(
            Units.rotationsToDegrees(this.encoder1.getPosition()),
            Units.rotationsToDegrees(this.encoder2.getPosition()));

    this.turretMotor.setEncoderPosition(Degrees.of(this.absoluteAngle));

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
    inputs.targetAngle =
        calculator.getLastParameters() != null
            ? calculator.getLastParameters().turretAngle().getDegrees()
            : 0;
    inputs.atTarget = isAtTargetAngle();
    inputs.turret =
        RobotStates.getInstance()
            .getRobotPose()
            .transformBy(TurretConstants.Position.TURRET_OFFSET_FROM_ROBOT_CENTER)
            .plus(
                new Transform2d(
                    Translation2d.kZero, Rotation2d.fromDegrees(turret.getAngle().in(Degrees))));
    inputs.absoluteAngle = this.absoluteAngle;
    inputs.encoder1 = this.encoder1.getPosition();
    inputs.encoder2 = this.encoder2.getPosition();

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

  private double determineTurretAngleDegrees(double e1, double e2) {
    double difference = e2 - e1;
    if (difference > 250) {
      difference -= 360;
    }
    if (difference < -250) {
      difference += 360;
    }
    difference *= TurretConstants.SLOPE;

    double e1Rotations =
        (difference * TurretConstants.GEAR_0_TOOTH_COUNT / TurretConstants.GEAR_1_TOOTH_COUNT)
            / 360.0;
    double e1RotationsFloored = Math.floor(e1Rotations);
    double turretAngle =
        (e1RotationsFloored * 360.0 + e1)
            * (TurretConstants.GEAR_1_TOOTH_COUNT / TurretConstants.GEAR_0_TOOTH_COUNT);
    if (turretAngle - difference < -100) {
      turretAngle +=
          TurretConstants.GEAR_1_TOOTH_COUNT / TurretConstants.GEAR_0_TOOTH_COUNT * 360.0;
    } else if (turretAngle - difference > 100) {
      turretAngle -=
          TurretConstants.GEAR_1_TOOTH_COUNT / TurretConstants.GEAR_0_TOOTH_COUNT * 360.0;
    }
    return -turretAngle;
  }
}
