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
import edu.wpi.first.units.measure.Voltage;
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
  private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();

  private final SparkFlex rollerMasterFlex =
      new SparkFlex(IntakeConstants.ID_intakeLeaderRollerMotor, MotorType.kBrushless);
  private final SparkFlex rollerFollowerFlex =
      new SparkFlex(IntakeConstants.ID_intakeFollowerRollerMotor, MotorType.kBrushless);

  private final SmartMotorControllerConfig rollerMotorConfig =
      new SmartMotorControllerConfig(this)
          .withMechanismCircumference(IntakeConstants.roller.MECHANISM_CIRCUNFERENCE)
          .withFollowers(Pair.of(rollerFollowerFlex, true))
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
  ;

  private final SmartMotorController rollerMotor =
      new SparkWrapper(rollerMasterFlex, DCMotor.getNeoVortex(2), rollerMotorConfig);

  private final FlyWheelConfig rollerConfig =
      new FlyWheelConfig(rollerMotor)
          // Diameter of the flywheel.
          .withDiameter(Inches.of(4))
          // Mass of the flywheel.
          .withMass(Pounds.of(1))
          .withMOI(KilogramSquareMeters.of(0.003))
          .withTelemetry("RollerMech", TelemetryVerbosity.HIGH)
          .withMechanismPositionConfig(IntakeConstants.roller.robotToMechanism);

  private final FlyWheel roller = new FlyWheel(rollerConfig);

  public RollerSubsystem() {}

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Subsystems/Intake", rollerInputs);
    roller.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    roller.simIterate();
  }

  public void updateInputs() {
    rollerInputs.rollerVoltage = rollerMotor.getVoltage();
    rollerInputs.rollerCurrent = rollerMotor.getStatorCurrent();
    rollerInputs.rollerPosition = rollerMotor.getMeasurementPosition();
    rollerInputs.rollerVelocity = rollerMotor.getMeasurementVelocity();
  }

  public Command runRoller() {
    return this.setDutyCycle(IntakeConstants.roller.setpoints.INTAKE_DUTY_CYCLE);
  }

  public Command runRollerReversed() {
    return this.setDutyCycle(IntakeConstants.roller.setpoints.REVERSE_DUTY_CYCLE);
  }

  public Command stopRoller() {
    return this.setDutyCycle(IntakeConstants.roller.setpoints.STOP_DUTY_CYCLE);
  }

  private Command setVoltage(Voltage voltage) {
    return roller.setVoltage(voltage);
  }

  private Command setDutyCycle(double dutycycle) {
    return roller.set(dutycycle);
  }
}
