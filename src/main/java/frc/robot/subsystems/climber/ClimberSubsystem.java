package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ClimberSubsystem extends SubsystemBase {
  private static ClimberSubsystem instance;

  public static ClimberSubsystem getInstance() {
    if (instance == null) {
      instance = new ClimberSubsystem();
    }
    return instance;
  }

  private final ClimberInputsAutoLogged climberInputs;

  private final SparkFlex climberFlex;

  private final SmartMotorControllerConfig climberMotorConfig;

  private final SmartMotorController climberMotor;

  private final ElevatorConfig climberConfig;

  private final Elevator climber;

  public ClimberSubsystem() {
    this.climberInputs = new ClimberInputsAutoLogged();
    this.climberMotorConfig =
        new SmartMotorControllerConfig(this)
            .withIdleMode(ClimberConstants.MOTOR_MODE)
            .withGearing(ClimberConstants.GEARING)
            .withMotorInverted(ClimberConstants.INVERTED)
            .withStatorCurrentLimit(ClimberConstants.STATOR_CURRENT_LIMIT)
            .withMechanismCircumference(ClimberConstants.MECHANISM_CIRCUNFERENCE)
            .withSoftLimit(ClimberConstants.CLIMBER_MIN_HEIGHT, ClimberConstants.CLIMBER_MAX_HEIGHT)
            .withTelemetry(TelemetryVerbosity.MID)
            .withClosedLoopController(
                new PIDController(
                    ClimberConstants.FeedBackController.kP,
                    ClimberConstants.FeedBackController.kI,
                    ClimberConstants.FeedBackController.kD))
            .withSimClosedLoopController(
                new PIDController(
                    ClimberConstants.FeedBackController.SIM.kP,
                    ClimberConstants.FeedBackController.SIM.kI,
                    ClimberConstants.FeedBackController.SIM.kD))
            .withFeedforward(
                new SimpleMotorFeedforward(
                    ClimberConstants.FeedForwardController.kS,
                    ClimberConstants.FeedForwardController.kS,
                    ClimberConstants.FeedForwardController.kA))
            .withFeedforward(
                new SimpleMotorFeedforward(
                    ClimberConstants.FeedForwardController.SIM.kS,
                    ClimberConstants.FeedForwardController.SIM.kS,
                    ClimberConstants.FeedForwardController.SIM.kA))
            .withControlMode(ClimberConstants.CONTROL_MODE);

    this.climberFlex =
        new SparkFlex(ClimberConstants.CLIMBER_MOTOR_ID, ClimberConstants.MOTOR_TYPE);

    this.climberMotor =
        new SparkWrapper(climberFlex, ClimberConstants.DC_MOTOR, climberMotorConfig);

    this.climberConfig =
        new ElevatorConfig(climberMotor)
            .withHardLimits(
                ClimberConstants.CLIMBER_MIN_HEIGHT, ClimberConstants.CLIMBER_MAX_HEIGHT)
            .withStartingHeight(ClimberConstants.STARTING_HEIGHT)
            .withMass(ClimberConstants.CLIMBER_MASS);

    this.climber = new Elevator(climberConfig);
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Subsystems/Climber", climberInputs);
    climber.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    climber.simIterate();
  }

  public Command climb() {
    return climber.setHeight(Meters.of(0.176));
  }

  public Command defaultPosition() {
    return climber.setHeight(Meters.of(0));
  }

  public boolean isClimbed() {
    return climber
        .getHeight()
        .isNear(
            climber.getMechanismSetpoint().isEmpty()
                ? Meters.of(0)
                : Meters.of(
                    climber.getMechanismSetpoint().get().in(Rotations)
                        * ClimberConstants.MECHANISM_CIRCUNFERENCE.in(Meters)),
            ClimberConstants.HEIGHT_ERROR_ALLOWED);
  }

  public Command setDutyCycle(double dutycycle) {
    return climber.set(dutycycle);
  }

  public void updateInputs() {
    climberInputs.climberAppliedVolts = climberMotor.getVoltage();
    climberInputs.climberCurrent = climberMotor.getStatorCurrent();
    climberInputs.climberPosition = climber.getHeight();
    climberInputs.climberSetpoint =
        climber.getMechanismSetpoint().isEmpty()
            ? Meters.of(0)
            : Meters.of(
                climber.getMechanismSetpoint().get().in(Rotations)
                    * ClimberConstants.MECHANISM_CIRCUNFERENCE.in(Meters));
    ;
    climberInputs.climberVelocity = climber.getVelocity();
  }

  public SparkAbsoluteEncoder getAbsoluteEncoder() {
    return climberFlex.getAbsoluteEncoder();
  }
}
