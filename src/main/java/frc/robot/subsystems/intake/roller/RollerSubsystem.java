package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.intake.IntakeConstants;
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

public class RollerSubsystem extends SubsystemBase {
  RollerInputsAutoLogged rollerInputs;

  SparkFlex rollerMasterMotor;
  SparkFlex rollerFollowerMotor;

  SmartMotorControllerConfig smartRollerMotorConfig;
          
  SmartMotorController smartRollerMotor;

  FlyWheelConfig rollerMechanismConfig;

  FlyWheel rollerMechanism;

  public RollerSubsystem() {
    this.rollerMasterMotor =
      new SparkFlex(IntakeConstants.ID_intakeLeaderRollerMotor, MotorType.kBrushless);
    this.rollerFollowerMotor =
        new SparkFlex(IntakeConstants.ID_intakeFollowerRollerMotor, MotorType.kBrushless);

    this.smartRollerMotorConfig =
      new SmartMotorControllerConfig(this)
          .withMechanismCircumference(IntakeConstants.roller.MECHANISM_CIRCUNFERENCE)
          .withFollowers(Pair.of(rollerFollowerMotor, true))
          .withClosedLoopController(
              new PIDController(
                  IntakeConstants.roller.PID.kP,
                  IntakeConstants.roller.PID.kI,
                  IntakeConstants.roller.PID.kD))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(2)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("RollerMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25));
          
    this.smartRollerMotor =
        new SparkWrapper(this.rollerMasterMotor, DCMotor.getNeoVortex(2), this.smartRollerMotorConfig);

    this.rollerMechanismConfig =
        new FlyWheelConfig(this.smartRollerMotor)
            .withDiameter(Inches.of(4))
            .withMass(Pounds.of(1))
            .withMOI(KilogramSquareMeters.of(0.003))
            .withTelemetry("RollerMech", TelemetryVerbosity.HIGH)
            .withMechanismPositionConfig(IntakeConstants.roller.robotToMechanism);

    this.rollerMechanism = new FlyWheel(this.rollerMechanismConfig);
  }

  @Override
  public void periodic() {
    updateInputs();
    this.rollerMechanism.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    this.rollerMechanism.simIterate();
  }

  private void updateInputs() {
    this.rollerInputs.rollerVoltage = this.smartRollerMotor.getVoltage();
    this.rollerInputs.rollerCurrent = this.smartRollerMotor.getStatorCurrent();
    this.rollerInputs.rollerPosition = this.smartRollerMotor.getMeasurementPosition();
    this.rollerInputs.rollerVelocity = this.smartRollerMotor.getMeasurementVelocity();

    Logger.processInputs("Subsystems/Intake/Roller", rollerInputs);
  }

  public Command setVelocity(AngularVelocity velocity) {
    return this.rollerMechanism.setSpeed(velocity);
  }
}
