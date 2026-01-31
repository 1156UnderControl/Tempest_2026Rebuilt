// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.hopper.KickerConstants;
import frc.robot.constants.intake.IntakeConstants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class KickerSubsystem extends SubsystemBase {
  private static KickerSubsystem instance;

  public static KickerSubsystem getInstance() {
    if (instance == null) {
      instance = new KickerSubsystem();
    }
    return instance;
  }

  private final SparkFlex kickerFlex =
      new SparkFlex(KickerConstants.MOTOR_ID, KickerConstants.MOTOR_TYPE);
  private final KickerInputsAutoLogged kickerInputs = new KickerInputsAutoLogged();

  private SmartMotorControllerConfig smartKickerMotorConfig =
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
          .withMotorInverted(KickerConstants.Position.MOTOR_INVERTED)
          .withTelemetry(KickerConstants.MOTOR_NAME, KickerConstants.TELEMETRY);

  private final SmartMotorController kickerMotor =
      new SparkWrapper(kickerFlex, KickerConstants.MOTOR, smartKickerMotorConfig);

  private final FlyWheelConfig kickerMechanismConfig =
      new FlyWheelConfig(kickerMotor)
          .withDiameter(KickerConstants.Wheel.DIAMETER)
          .withMass(KickerConstants.Wheel.MASS)
          .withMOI(KickerConstants.Wheel.MOI)
          .withTelemetry(KickerConstants.MOTOR_NAME, KickerConstants.TELEMETRY)
          .withMechanismPositionConfig(IntakeConstants.roller.robotToMechanism);

  private final FlyWheel kicker = new FlyWheel(kickerMechanismConfig);

  public KickerSubsystem() {}

  @Override
  public void periodic() {
    this.updateInputs();
    kicker.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    this.kicker.simIterate();
  }

  public void updateInputs() {
    kickerInputs.spindexerAppliedVolts = kickerMotor.getVoltage();
    kickerInputs.spindexerCurrent = kickerMotor.getStatorCurrent();
    kickerInputs.spindexerPosition = kickerMotor.getMeasurementPosition();
    kickerInputs.spindexerVelocity = kickerMotor.getMeasurementVelocity();
  }

  public Command setDutyCycle(double dutycycle) {
    return kicker.set(dutycycle);
  }
}
