package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStates;
import frc.robot.subsystems.shooter.ShooterCalculator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystem extends SubsystemBase {

  private final ShooterCalculator calculator;
  private final TurretInputsAutoLogged inputs;
  private static TurretSubsystem instance;

  private final TalonFX turretTalon;
  private final SmartMotorControllerConfig smartTurretMotorConfig;
  private final SmartMotorController turretMotor;
  private final PivotConfig turretMechanismConfig;
  private final Pivot turret;

  private PositionTorqueCurrentFOC torqueCurrentControl =
      new PositionTorqueCurrentFOC(Degrees.of(0));

  private double absoluteAngle;

  private Angle setpoint = Degrees.of(0);

  private TurretSubsystem(SparkAbsoluteEncoder climberEncoder) {
    calculator = ShooterCalculator.getInstance();
    smartTurretMotorConfig =
        new SmartMotorControllerConfig(this)
            .withIdleMode(TurretConstants.MOTOR_MODE)
            .withGearing(TurretConstants.GEARING)
            .withMotorInverted(TurretConstants.INVERTED)
            .withSoftLimit(TurretConstants.TURRET_MIN_ANGLE, TurretConstants.TURRET_MAX_ANGLE)
            .withStatorCurrentLimit(TurretConstants.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(TurretConstants.SUPPLY_CURRENT_LIMIT)
            .withClosedLoopController(
                TurretConstants.FeedBackController.kP,
                TurretConstants.FeedBackController.kI,
                TurretConstants.FeedBackController.kD,
                TurretConstants.MAX_VELOCITY,
                TurretConstants.MAX_ACCELERATION)
            .withSimClosedLoopController(
                TurretConstants.FeedBackController.SIM.kP,
                TurretConstants.FeedBackController.SIM.kI,
                TurretConstants.FeedBackController.SIM.kD,
                TurretConstants.MAX_VELOCITY,
                TurretConstants.MAX_ACCELERATION)
            .withFeedforward(
                new SimpleMotorFeedforward(
                    TurretConstants.FeedForwardController.kS,
                    TurretConstants.FeedForwardController.kV,
                    TurretConstants.FeedForwardController.kA))
            .withSimFeedforward(
                new SimpleMotorFeedforward(
                    TurretConstants.FeedForwardController.SIM.kS,
                    TurretConstants.FeedForwardController.SIM.kV,
                    TurretConstants.FeedForwardController.SIM.kA))
            .withTelemetry(TelemetryVerbosity.MID)
            .withControlMode(TurretConstants.CONTROL_MODE)
            .withStartingPosition(TurretConstants.STARTING_ANGLE);

    turretTalon = new TalonFX(TurretConstants.TURRET_MOTOR_ID);

    turretMotor = new TalonFXWrapper(turretTalon, TurretConstants.DC_MOTOR, smartTurretMotorConfig);

    turretMechanismConfig =
        new PivotConfig(turretMotor)
            .withHardLimit(TurretConstants.TURRET_MIN_ANGLE, TurretConstants.TURRET_MAX_ANGLE)
            .withMOI(TurretConstants.TURRET_SIZE, TurretConstants.TURRET_MASS);

    turret = new Pivot(turretMechanismConfig);
    // this.absoluteAngle =
    // determineTurretAngleDegrees(
    // Units.rotationsToDegrees(this.encoder1.getPosition()),
    // Units.rotationsToDegrees(this.encoder2.getPosition()));

    // this.turretMotor.setEncoderPosition(Degrees.of(this.absoluteAngle));

    inputs = new TurretInputsAutoLogged();
  }

  public static TurretSubsystem getInstance(SparkAbsoluteEncoder climberEncoder) {
    if (instance == null) {
      instance = new TurretSubsystem(climberEncoder);
    }
    return instance;
  }

  @Override
  public void periodic() {
    if (TurretConstants.USE_TORQUE_CURRENT) {
      this.setpoint = this.torqueCurrentControl.getPositionMeasure();
    } else {
      this.setpoint =
          turret.getMechanismSetpoint().isEmpty()
              ? Degrees.of(0)
              : turret.getMechanismSetpoint().get();
    }
    turret.updateTelemetry();
    updateInputs();
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }

  private void updateInputs() {
    inputs.currentAngle = turret.getAngle().in(Degrees);
    inputs.targetAngle = this.setpoint.in(Degrees);
    inputs.atTarget = isAtTargetAngle();
    inputs.current = turretMotor.getStatorCurrent();
    inputs.voltage = turretMotor.getVoltage();
    inputs.turret =
        RobotStates.getInstance()
            .getRobotPose()
            .transformBy(TurretConstants.TURRET_OFFSET_FROM_ROBOT_CENTER)
            .plus(
                new Transform2d(
                    Translation2d.kZero, Rotation2d.fromDegrees(turret.getAngle().in(Degrees))));
    inputs.absoluteAngle = this.absoluteAngle;
    Logger.processInputs("Subsystems/Shooter/Turret/", inputs);
  }

  public Command trackHub() {
    return this.setTurretToDegrees(() -> calculator.getParameters(true).turretAngle().getDegrees());
  }

  public Command trackPass() {
    return this.setTurretToDegrees(
        () -> calculator.getParameters(false).turretAngle().getDegrees());
  }

  public Command trackNonOdometry() {
    return this.setTurretToDegrees(() -> 180.0);
  }

  public boolean isAtTargetAngle() {
    return turret.getAngle().isNear(setpoint, TurretConstants.ANGLE_ERROR_ALLOWED);
  }

  private Command setTurretToDegrees(Supplier<Double> setpoint) {
    if (TurretConstants.USE_TORQUE_CURRENT) {
      return this.run(
          () ->
              turretTalon.setControl(
                  torqueCurrentControl.withPosition(
                      Degrees.of(
                          getClosestTargetAngle(setpoint.get(), turret.getAngle().in(Degrees))))));
    }
    return turret.setAngle(
        Degrees.of(getClosestTargetAngle(setpoint.get(), turret.getAngle().in(Degrees))));
  }

  private double getClosestTargetAngle(double targetAngle, double currentAngle) {
    double delta = (targetAngle - currentAngle) % 360;

    if (delta > 180) {
      delta -= 360;
    } else if (delta < -180) {
      delta += 360;
    }

    double potentialTarget = currentAngle + delta;

    if (potentialTarget > TurretConstants.TURRET_MAX_ANGLE.in(Degrees)) {
      potentialTarget -= 360;
    } else if (potentialTarget < TurretConstants.TURRET_MIN_ANGLE.in(Degrees)) {
      potentialTarget += 360;
    }

    return potentialTarget;
  }
}
