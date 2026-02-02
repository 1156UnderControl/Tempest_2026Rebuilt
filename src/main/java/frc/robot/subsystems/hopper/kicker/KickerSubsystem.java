// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper.kicker;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.hopper.KickerConstants;
import frc.robot.constants.intake.IntakeConstants;
import frc.robot.subsystems.hopper.KickerInputsAutoLogged;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class KickerSubsystem extends SubsystemBase {
  private static KickerSubsystem instance;

  private final KickerInputsAutoLogged kickerInputs;
  private final SparkFlex kickerMotor;
  private SmartMotorControllerConfig smartKickerMotorConfig;
  private final SmartMotorController smartKickerMotor;
  private final FlyWheelConfig kickerMechanismConfig;
  private final FlyWheel kickerMechanism;

  private KickerSubsystem() {
    this.kickerInputs = new KickerInputsAutoLogged();

    this.kickerMotor = new SparkFlex(KickerConstants.MOTOR_ID, KickerConstants.MOTOR_TYPE);

    this.smartKickerMotorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(KickerConstants.CONTROL_MODE)
          .withClosedLoopController(
              KickerConstants.ClosedLoopController.kP,
              KickerConstants.ClosedLoopController.kI,
              KickerConstants.ClosedLoopController.kD,
              KickerConstants.Velocity.MAX_VELOCITY,
              KickerConstants.Velocity.ACCELERATION)
          .withGearing(KickerConstants.MOTOR_GEARING)
          .withIdleMode(KickerConstants.MOTOR_MODE)
          .withMechanismCircumference(Inches.of(2))
          .withMotorInverted(true)
          .withTelemetry(KickerConstants.MOTOR_NAME, KickerConstants.TELEMETRY);

    this.smartKickerMotor =
      new SparkWrapper(kickerMotor, KickerConstants.MOTOR, smartKickerMotorConfig);

    this.kickerMechanismConfig =
      new FlyWheelConfig(this.smartKickerMotor)
          .withDiameter(KickerConstants.Wheel.DIAMETER)
          .withMass(KickerConstants.Wheel.MASS)
          .withMOI(KickerConstants.Wheel.MOI)
          .withTelemetry(KickerConstants.MOTOR_NAME, KickerConstants.TELEMETRY)
          .withMechanismPositionConfig(IntakeConstants.roller.robotToMechanism);

    this.kickerMechanism = new FlyWheel(this.kickerMechanismConfig);
  }

  public static KickerSubsystem getInstance() {
    if (instance == null) {
      instance = new KickerSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    this.updateInputs();
    this.kickerMechanism.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    this.kickerMechanism.simIterate();
  }

  public void updateInputs() {
    kickerInputs.kickerAppliedVolts = smartKickerMotor.getVoltage();
    kickerInputs.kickerCurrent = smartKickerMotor.getStatorCurrent();
    kickerInputs.kickerPosition = smartKickerMotor.getMeasurementPosition();
    kickerInputs.kickerVelocity = smartKickerMotor.getMechanismVelocity();
    kickerInputs.kickerTargetVelocity = smartKickerMotor.getMechanismSetpointVelocity().isEmpty() ? RotationsPerSecond.of(0) : smartKickerMotor.getMechanismSetpointVelocity().get();
  }

  public Command setVelocity(AngularVelocity velocity) {
    return kickerMechanism.setSpeed(velocity);
  }
}
