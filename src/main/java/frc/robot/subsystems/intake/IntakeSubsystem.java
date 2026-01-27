package frc.robot.subsystems.intake;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.intake.IntakeConstants;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Elevator;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeInputsAutoLogged intakeInputs = new IntakeInputsAutoLogged();

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

  private final SparkFlex extensorSparkFlex =
      new SparkFlex(IntakeConstants.ID_intakeFollowerRollerMotor, MotorType.kBrushless);

  private final SmartMotorControllerConfig extensorMotorConfig =
      new SmartMotorControllerConfig(this)
          .withMechanismCircumference(IntakeConstants.extensor.MECHANISM_CIRCUNFERENCE)
          .withClosedLoopController(
              new PIDController(
                  IntakeConstants.roller.PID.kP,
                  IntakeConstants.roller.PID.kI,
                  IntakeConstants.roller.PID.kD))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("ExtensorMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController extensorMotor =
      new SparkWrapper(extensorSparkFlex, DCMotor.getNeoVortex(1), extensorMotorConfig);

  private final ElevatorConfig extensorConfig =
      new ElevatorConfig(rollerMotor)
          .withMass(Pounds.of(1))
          .withHardLimits(
              IntakeConstants.limits.MIN_EXTENSION, IntakeConstants.limits.MAX_EXTENSION)
          .withTelemetry("ExtensorMech", TelemetryVerbosity.HIGH)
          .withMechanismPositionConfig(IntakeConstants.extensor.robotToMechanism);

  private final Elevator extensor = new Elevator(extensorConfig);

  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Subsystems/Intake", intakeInputs);
    roller.updateTelemetry();
    extensor.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    roller.simIterate();
    extensor.simIterate();
  }

  public void updateInputs() {
    intakeInputs.extensorVoltage = extensorMotor.getVoltage();
    intakeInputs.extensorCurrent = extensorMotor.getStatorCurrent();
    intakeInputs.extensorPosition = extensorMotor.getMeasurementPosition();
    intakeInputs.extensorVelocity = extensorMotor.getMeasurementVelocity();

    intakeInputs.rollerVoltage = rollerMotor.getVoltage();
    intakeInputs.rollerCurrent = rollerMotor.getStatorCurrent();
    intakeInputs.rollerPosition = rollerMotor.getMeasurementPosition();
    intakeInputs.rollerVelocity = rollerMotor.getMeasurementVelocity();
  }
}
