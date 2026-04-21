package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.java_is_undercontrol.util.StabilizeChecker;
import frc.robot.RobotStates;
import frc.robot.subsystems.shooter.ShooterCalculator;
import java.util.function.Supplier;
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
  private final TalonFX followerMotor;
  private final SmartMotorController smartFlywheelMotor;
  private final FlyWheelConfig flyWheelConfig;
  private final FlyWheel flywheelMechanism;

  private final FlyWheelInputsAutoLogged inputs;

  private static FlyWheelSubsystem instance;

  private VelocityTorqueCurrentFOC torqueCurrentRequest = new VelocityTorqueCurrentFOC(RPM.of(0));

  private double flyWheelRPM;

  public boolean isUsingFeedDeadband = false;

  private AngularVelocity setpoint = RPM.of(0);

  private StabilizeChecker isNotAtSetpointChecker = new StabilizeChecker(0.2);

  private TalonFXConfiguration talonConfigurationLead = new TalonFXConfiguration();

  private TalonFXConfiguration talonConfigurationFollower = new TalonFXConfiguration();

  private TalonFXConfigurator talonConfiguratorLead;

  private TalonFXConfigurator talonConfiguratorFollower;

  private FlyWheelSubsystem() {
    this.calculator = ShooterCalculator.getInstance();
    this.leadMotor = new TalonFX(19);
    this.followerMotor = new TalonFX(18);

    this.motorConfig =
        new SmartMotorControllerConfig(this)
            .withMotorInverted(true)
            .withFollowers(Pair.of(this.followerMotor, true))
            .withIdleMode(MotorMode.COAST)
            .withGearing(FlyWheelsConstants.MOTOR_GEARING)
            .withSupplyCurrentLimit(Amps.of(40
            ))
            .withStatorCurrentLimit(Amps.of(120))
            .withClosedLoopController(
                FlyWheelsConstants.ClosedLoopController.kP,
                FlyWheelsConstants.ClosedLoopController.kI,
                FlyWheelsConstants.ClosedLoopController.kD)
            .withSimClosedLoopController(4, 0, 0)
            .withTelemetry(TelemetryVerbosity.MID)
            .withSimFeedforward(new SimpleMotorFeedforward(0.19, 0.11, 0))
            .withControlMode(ControlMode.CLOSED_LOOP);

    this.torqueCurrentRequest.withFeedForward(FlyWheelsConstants.FEEDFORWARD_TORQUE_CURRENT_A);

    this.talonConfiguratorLead = leadMotor.getConfigurator();
    this.talonConfiguratorFollower = followerMotor.getConfigurator();

    this.smartFlywheelMotor =
        new TalonFXWrapper(this.leadMotor, FlyWheelsConstants.FLYWHEEL_MOTOR, this.motorConfig);

    this.flyWheelConfig =
        new FlyWheelConfig(smartFlywheelMotor)
            .withMOI(FlyWheelsConstants.FLYWHEEL_SIZE, FlyWheelsConstants.FLYWHEEL_MASS);

    this.flywheelMechanism = new FlyWheel(flyWheelConfig);

    this.leadMotor.getVelocity().setUpdateFrequency(250);

    talonConfigurationFollower.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    talonConfiguratorFollower.apply(talonConfigurationFollower.MotorOutput);

    talonConfiguratorLead.refresh(talonConfigurationLead.Slot0);
    talonConfiguratorLead.apply(talonConfigurationLead.Slot0.withKS(3.3).withKV(0.015));

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
    this.inputs.currentVelocity = this.smartFlywheelMotor.getMechanismVelocity().in(RPM);
    this.setpoint = torqueCurrentRequest.getVelocityMeasure();
    this.inputs.targetVelocity = setpoint.in(RPM);
    this.inputs.atTarget = isAtTargetVelocity();
    Logger.processInputs("Subsystems/Shooter/FlyWheels/", inputs);
  }

  public Command runLowVelocity() {
    return this.runFlywheel(() -> RPM.of(1000));
  }

  public Command stoppedVelocity() {
    return this.runFlywheel(() -> RPM.of(0));
  }

  public Command shootHub() {
    return this.runFlywheel(
        () -> RPM.of(ShooterCalculator.getInstance().getParameters(true).flywheelSpeed()));
  }

  public Command shootPass() {
    return this.runFlywheel(
        () -> RPM.of(ShooterCalculator.getInstance().getParameters(false).flywheelSpeed()));
  }

  public Command shootNonOdometry() {
    return this.runFlywheel(() -> RPM.of(2700.0));
  }

  private Command runFlywheel(Supplier<AngularVelocity> velocity) {
    return this.run(() -> leadMotor.setControl(torqueCurrentRequest.withVelocity(velocity.get())));
  }

  public Command runVelocityTorqueControlCalibration() {
    return this.run(
        () -> leadMotor.setControl(torqueCurrentRequest.withVelocity(this.flyWheelRPM / 60)));
  }

  public boolean isAtTargetVelocity() {
    return !isNotAtSetpointChecker.isStableInCondition(
            () -> {
              if (!RobotStates.getInstance().isOnAllianceZone()) {
                isUsingFeedDeadband = true;
                return !flywheelMechanism
                    .getSpeed()
                    .isNear(setpoint, FlyWheelsConstants.Velocity.VELOCITY_ERROR_ALLOWED_FOR_FEED);
              } else {
                isUsingFeedDeadband = false;
                return !flywheelMechanism
                    .getSpeed()
                    .isNear(setpoint, FlyWheelsConstants.Velocity.VELOCITY_ERROR_ALLOWED);
              }
            })
        && flywheelMechanism
            .getSpeed()
            .isNear(setpoint, FlyWheelsConstants.Velocity.VELOCITY_ERROR_ALLOWED_FIRST_SHOT);
  }
}
