package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.shooter.FlyWheelsConstants;
import frc.robot.subsystems.shooter.ShooterCalculator;
import org.littletonrobotics.junction.Logger;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlyWheelSubsystem extends SubsystemBase {

  private final ShooterCalculator calculator;

  private final SmartMotorControllerConfig motorConfig;
  private final TalonFX leadMotor;
  private final SmartMotorController smartFlywheelMotor;
  private final FlyWheelConfig flyWheelConfig;
  private final FlyWheel flywheelMechanism;

  private final FlyWheelInputsAutoLogged inputs;

  private static FlyWheelSubsystem instance;

  private FlyWheelSubsystem() {
    this.calculator = ShooterCalculator.getInstance();

    this.motorConfig =
        new SmartMotorControllerConfig(this)
            .withMotorInverted(true)
            .withFollowers(Pair.of(new TalonFX(18, CANBus.roboRIO()), true))
            .withIdleMode(MotorMode.BRAKE)
            .withGearing(FlyWheelsConstants.MOTOR_GEARING)
            .withTelemetry("FlyWheelLeadMotor", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(60))
            .withClosedLoopRampRate(Milliseconds.of(100))
            .withClosedLoopController(
                new ExponentialProfilePIDController(
                    new PIDController(
                        FlyWheelsConstants.ClosedLoopController.kP,
                        FlyWheelsConstants.ClosedLoopController.kI,
                        FlyWheelsConstants.ClosedLoopController.kD),
                    ExponentialProfilePIDController.createFlywheelConstraints(
                        FlyWheelsConstants.MAX_VOLTS,
                        FlyWheelsConstants.FLYWHEEL_MOTOR,
                        FlyWheelsConstants.FLYWHEEL_MASS,
                        FlyWheelsConstants.FLYWHEEL_SIZE,
                        FlyWheelsConstants.MOTOR_GEARING)))
            .withSimClosedLoopController(
                new ExponentialProfilePIDController(
                    new PIDController(
                        FlyWheelsConstants.ClosedLoopController.kP,
                        FlyWheelsConstants.ClosedLoopController.kI,
                        FlyWheelsConstants.ClosedLoopController.kD),
                    ExponentialProfilePIDController.createFlywheelConstraints(
                        FlyWheelsConstants.MAX_VOLTS,
                        FlyWheelsConstants.FLYWHEEL_MOTOR,
                        FlyWheelsConstants.FLYWHEEL_MASS,
                        FlyWheelsConstants.FLYWHEEL_SIZE,
                        FlyWheelsConstants.MOTOR_GEARING)))
            .withFeedforward(new SimpleMotorFeedforward(0.19, 0.11, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(0.19, 0.11, 0))
            .withControlMode(ControlMode.CLOSED_LOOP);

    this.leadMotor = new TalonFX(19);

    this.smartFlywheelMotor =
        new TalonFXWrapper(this.leadMotor, FlyWheelsConstants.FLYWHEEL_MOTOR, this.motorConfig);

    this.flyWheelConfig =
        new FlyWheelConfig(smartFlywheelMotor)
            .withMOI(FlyWheelsConstants.FLYWHEEL_SIZE, FlyWheelsConstants.FLYWHEEL_MASS);

    this.flywheelMechanism = new FlyWheel(flyWheelConfig);

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
    this.flywheelMechanism.updateTelemetry();
    this.updateInputs();
  }

  @Override
  public void simulationPeriodic() {
    this.flywheelMechanism.simIterate();
  }

  private void updateInputs() {
    this.inputs.currentVelocity = this.smartFlywheelMotor.getRotorVelocity().in(RotationsPerSecond);
    this.inputs.targetVelocity =
        this.smartFlywheelMotor.getMechanismSetpointVelocity().isEmpty()
            ? 0
            : this.smartFlywheelMotor.getMechanismSetpointVelocity().get().in(RotationsPerSecond);
    this.inputs.atTarget = isAtTargetVelocity();

    Logger.processInputs("Subsystems/Shooter/FlyWheels/", inputs);
  }

  public Command runVelocity() {
    return this.flywheelMechanism.setSpeed(RPM.of(4800));
  }

  public Command stoppedVelocity() {
    return this.flywheelMechanism.setSpeed(RotationsPerSecond.of(0));
  }

  public boolean isAtTargetVelocity() {
    return this.flywheelMechanism
        .getSpeed()
        .isNear(
            this.smartFlywheelMotor.getMechanismSetpointVelocity().isEmpty() == true
                ? RotationsPerSecond.of(0)
                : this.smartFlywheelMotor.getMechanismSetpointVelocity().get(),
            FlyWheelsConstants.Velocity.VELOCITY_ERROR_ALLOWED);
  }
}
