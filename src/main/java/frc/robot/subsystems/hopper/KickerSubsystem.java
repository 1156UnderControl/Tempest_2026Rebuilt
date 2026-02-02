// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

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

  private AngularVelocity setpoint = RPM.of(0);

  private final SparkFlex kickerFlex =
      new SparkFlex(KickerConstants.MOTOR_ID, MotorType.kBrushless);
  private final KickerInputsAutoLogged kickerInputs = new KickerInputsAutoLogged();

  private SmartMotorControllerConfig smartKickerMotorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(0, 0, 0, RPM.of(3000), RotationsPerSecondPerSecond.of(150))
          .withSimClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(2.5)))
          .withIdleMode(MotorMode.COAST)
          .withMechanismCircumference(Inches.of(2))
          .withMotorInverted(true)
          .withStatorCurrentLimit(Amps.of(50))
          .withTelemetry("KickerMotor", TelemetryVerbosity.HIGH);

  private final SmartMotorController kickerMotor =
      new SparkWrapper(kickerFlex, DCMotor.getNeoVortex(1), smartKickerMotorConfig);

  private final FlyWheelConfig kickerMechanismConfig =
      new FlyWheelConfig(kickerMotor)
          .withDiameter(Inches.of(4))
          .withMass(Kilograms.of(0.45))
          .withMOI(Inches.of(4), Kilograms.of(0.45))
          .withTelemetry("KickerMechanism", TelemetryVerbosity.HIGH)
          .withMechanismPositionConfig(KickerConstants.robotToMechanism);

  private final FlyWheel kicker = new FlyWheel(kickerMechanismConfig);

  public KickerSubsystem() {}

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
