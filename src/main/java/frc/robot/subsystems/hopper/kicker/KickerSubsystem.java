// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper.kicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class KickerSubsystem extends SubsystemBase {
  private static KickerSubsystem instance;

  public static KickerSubsystem getInstance() {
    if (instance == null) {
      instance = new KickerSubsystem();
    }
    return instance;
  }

  private final KickerInputsAutoLogged kickerInputs;

  private AngularVelocity setpoint;

  private final TalonFX kickerTalon;

  private SmartMotorControllerConfig smartKickerMotorConfig;

  private final SmartMotorController kickerMotor;

  private final FlyWheelConfig kickerMechanismConfig;

  private final FlyWheel kicker;

  private KickerSubsystem() {
    kickerInputs = new KickerInputsAutoLogged();

    kickerTalon = new TalonFX(16);

    smartKickerMotorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(0, 0, 0, RPM.of(3000), RotationsPerSecondPerSecond.of(150))
            .withSimClosedLoopController(12, 0, 0, RPM.of(0), RotationsPerSecondPerSecond.of(0))
            .withFeedforward(new SimpleMotorFeedforward(0, 0.26))
            .withSimFeedforward(new SimpleMotorFeedforward(0, 0))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(2.16)))
            .withIdleMode(MotorMode.COAST)
            .withMechanismCircumference(Inches.of(2))
            .withMotorInverted(true)
            .withSupplyCurrentLimit(Amps.of(20))
            .withStatorCurrentLimit(Amps.of(40));

    kickerMotor = new TalonFXWrapper(kickerTalon, DCMotor.getKrakenX60(1), smartKickerMotorConfig);

    kickerMechanismConfig =
        new FlyWheelConfig(kickerMotor)
            .withDiameter(KickerConstants.WHEEL_DIAMETER)
            .withMass(KickerConstants.WHEEL_MASS)
            .withMOI(KickerConstants.WHEEL_DIAMETER, KickerConstants.WHEEL_MASS)
            .withMechanismPositionConfig(
                new MechanismPositionConfig()
                    .withMaxRobotHeight(Meters.of(1.5))
                    .withMaxRobotLength(Meters.of(0.75))
                    .withRelativePosition(
                        new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5))));
    kicker = new FlyWheel(kickerMechanismConfig);

    setpoint = RPM.of(0);
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

  public Command expell() {
    return kicker.setSpeed(KickerConstants.setpoints.EXPEL);
  }

  public Command stop() {
    return kicker.setSpeed(KickerConstants.setpoints.STOPPED);
  }

  public Command setDutyCycle(double dutycycle) {
    return kicker.set(dutycycle);
  }

  public Trigger isAtVelocity() {
    return kicker.isNear(setpoint, KickerConstants.VELOCITY_ERROR_ALOWED);
  }
}
