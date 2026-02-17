package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.shooter.FlyWheelsConstants;
import frc.robot.subsystems.shooter.ShooterCalculator;
import org.littletonrobotics.junction.Logger;
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

  private VelocityTorqueCurrentFOC torqueCurrentRequest = new VelocityTorqueCurrentFOC(RPM.of(0));

  private double flyWheelRPM;

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
            .withClosedLoopController(
                FlyWheelsConstants.ClosedLoopController.kP,
                FlyWheelsConstants.ClosedLoopController.kI,
                FlyWheelsConstants.ClosedLoopController.kD)
            .withSimClosedLoopController(3, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(3.05, 0.0008, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(0.05, 0, 0))
            .withControlMode(ControlMode.CLOSED_LOOP);

    this.leadMotor = new TalonFX(19);

    this.smartFlywheelMotor =
        new TalonFXWrapper(this.leadMotor, FlyWheelsConstants.FLYWHEEL_MOTOR, this.motorConfig);

    this.flyWheelConfig =
        new FlyWheelConfig(smartFlywheelMotor)
            .withMOI(FlyWheelsConstants.FLYWHEEL_SIZE, FlyWheelsConstants.FLYWHEEL_MASS);

    this.flywheelMechanism = new FlyWheel(flyWheelConfig);

    this.inputs = new FlyWheelInputsAutoLogged();

    SmartDashboard.putNumber("Calibration/FlyWheelRPM", 0);
    this.flyWheelRPM = 0;
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
    this.flyWheelRPM = SmartDashboard.getNumber("Calibration/FlyWheelRPM", 0);
  }

  @Override
  public void simulationPeriodic() {
    this.flywheelMechanism.simIterate();
  }

  private void updateInputs() {
    this.inputs.currentVelocity = this.smartFlywheelMotor.getRotorVelocity().in(RPM);
    this.inputs.targetVelocity = torqueCurrentRequest.getVelocityMeasure().in(RPM);
    this.inputs.atTarget = isAtTargetVelocityShoot();

    Logger.processInputs("Subsystems/Shooter/FlyWheels/", inputs);
  }

  public Command runLowVelocity() {
    return this.runVelocityTorqueControl(RPM.of(2000));
  }

  public Command stoppedVelocity() {
    return this.runVelocityTorqueControl(RPM.of(0));
  }

  public Command shootHub() {
    return this.run(
        () -> {
          AngularVelocity velocity =
              RPM.of(ShooterCalculator.getInstance().getParameters(true).flywheelSpeed());
          leadMotor.setControl(torqueCurrentRequest.withVelocity(velocity));
        });
  }

  public Command shootPass() {
    return this.run(
        () -> {
          AngularVelocity velocity =
              RPM.of(ShooterCalculator.getInstance().getParameters(false).flywheelSpeed());
          leadMotor.setControl(torqueCurrentRequest.withVelocity(velocity));
        });
  }

  public Command shootNonOdometry() {
    return this.run(
        () -> {
          AngularVelocity velocity = RPM.of(2700.0);
          leadMotor.setControl(torqueCurrentRequest.withVelocity(velocity));
        });
  }

  public Command runVelocity() {
    return this.run(
        () -> {
          AngularVelocity velocity =
              RPM.of(ShooterCalculator.getInstance().getParameters(true).flywheelSpeed());
          leadMotor.setControl(torqueCurrentRequest.withVelocity(velocity));
        });
  }

  private Command runVelocityTorqueControl(AngularVelocity velocity) {
    return this.run(() -> leadMotor.setControl(torqueCurrentRequest.withVelocity(velocity)));
  }

  public Command runVelocityTorqueControlCalibration() {
    return this.run(
        () -> leadMotor.setControl(torqueCurrentRequest.withVelocity(this.flyWheelRPM / 60)));
  }

  public boolean isAtTargetVelocityShoot() {
    return this.flywheelMechanism
        .getSpeed()
        .isNear(
            RPM.of(ShooterCalculator.getInstance().getParameters(true).flywheelSpeed()),
            FlyWheelsConstants.Velocity.VELOCITY_ERROR_ALLOWED);
  }
}
