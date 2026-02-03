// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper.kicker;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.hopper.KickerConstants;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class KickerSubsystem extends SubsystemBase {
  private static KickerSubsystem instance;

  public static KickerSubsystem getInstance() {
    if (instance == null) {
      instance = new KickerSubsystem();
    }
    return instance;
  }

  private final KickerInputsAutoLogged kickerInputs = new KickerInputsAutoLogged();

  private AngularVelocity setpoint = RPM.of(0);

  private final SparkFlex kickerFlex;

  private SmartMotorControllerConfig smartKickerMotorConfig;

  private final SmartMotorController kickerMotor;

  private final FlyWheelConfig kickerMechanismConfig;

  private final FlyWheel kicker;

  public KickerSubsystem() {
    kickerFlex = new SparkFlex(KickerConstants.MOTOR_ID, MotorType.kBrushless);

    smartKickerMotorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                0,
                0,
                0,
                KickerConstants.Motor.CLOSED_LOOP_MAX_VELOCITY,
                KickerConstants.Motor.CLOSED_LOOP_MAX_ACCELERATION)
            .withSimClosedLoopController(
                50,
                0,
                0,
                KickerConstants.Motor.SIM_CLOSED_LOOP_MAX_VELOCITY,
                KickerConstants.Motor.SIM_CLOSED_LOOP_MAX_ACCELERATION)
            .withGearing(
                new MechanismGearing(GearBox.fromReductionStages(KickerConstants.Motor.GEAR_RATIO)))
            .withIdleMode(MotorMode.COAST)
            .withMechanismCircumference(KickerConstants.Motor.MECHANISM_CIRCUMFERENCE)
            .withMotorInverted(KickerConstants.Motor.INVERTED)
            .withStatorCurrentLimit(KickerConstants.Motor.STATOR_CURRENT_LIMIT)
            .withTelemetry(KickerConstants.Telemetry.MOTOR, TelemetryVerbosity.HIGH);

    kickerMotor = new SparkWrapper(kickerFlex, DCMotor.getNeoVortex(1), smartKickerMotorConfig);

    kickerMechanismConfig =
        new FlyWheelConfig(kickerMotor)
            .withDiameter(KickerConstants.Mechanism.WHEEL_DIAMETER)
            .withMass(KickerConstants.Mechanism.WHEEL_MASS)
            .withMOI(
                KickerConstants.Mechanism.WHEEL_DIAMETER, KickerConstants.Mechanism.WHEEL_MASS)
            .withTelemetry(KickerConstants.Telemetry.MECHANISM, TelemetryVerbosity.HIGH)
            .withMechanismPositionConfig(KickerConstants.robotToMechanism);
    kicker = new FlyWheel(kickerMechanismConfig);
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Subsystems/Kicker", kickerInputs);
    kicker.updateTelemetry();
    this.setpoint =
        kickerMotor.getMechanismSetpointVelocity().isEmpty()
            ? RPM.of(0)
            : kickerMotor.getMechanismSetpointVelocity().get();
  }

  @Override
  public void simulationPeriodic() {
    this.kicker.simIterate();
  }

  public void updateInputs() {
    kickerInputs.kickerAppliedVolts = kickerMotor.getVoltage();
    kickerInputs.kickerCurrent = kickerMotor.getStatorCurrent();
    kickerInputs.kickerPosition = kickerMotor.getMechanismPosition();
    kickerInputs.kickerVelocity = kickerMotor.getMechanismVelocity();
    kickerInputs.kickerSetpoint = this.setpoint;
  }

  public Command shoot() {
    return kicker.setSpeed(KickerConstants.setpoints.SHOOTING);
  }

  public Command stop() {
    return kicker.setSpeed(KickerConstants.setpoints.STOPPED);
  }

  public Command setDutyCycle(double dutycycle) {
    return kicker.set(dutycycle);
  }

  public Trigger isAtVelocity() {
    return kicker.isNear(setpoint, KickerConstants.DEADBAND_IS_AT);
  }
}
