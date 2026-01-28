package frc.robot.subsystems.intake.extensor;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.intake.IntakeConstants;
import org.littletonrobotics.junction.Logger;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ExtensorSubsystem extends SubsystemBase {
  private final ExtensorInputsAutoLogged extensorInputs = new ExtensorInputsAutoLogged();

  private final SparkFlex extensorSparkFlex =
      new SparkFlex(IntakeConstants.ID_intakeFollowerRollerMotor, MotorType.kBrushless);

  private final SmartMotorControllerConfig extensorMotorConfig =
      new SmartMotorControllerConfig(this)
          .withMechanismCircumference(IntakeConstants.extensor.MECHANISM_CIRCUNFERENCE)
          .withClosedLoopController(
              new ExponentialProfilePIDController(
                  IntakeConstants.roller.PID.kP,
                  IntakeConstants.roller.PID.kI,
                  IntakeConstants.roller.PID.kD,
                  ExponentialProfilePIDController.createElevatorConstraints(
                      IntakeConstants.extensor.maxVolts,
                      IntakeConstants.extensor.motor,
                      IntakeConstants.extensor.mass,
                      IntakeConstants.extensor.drumRadius,
                      IntakeConstants.extensor.gearing)))
          .withGearing(IntakeConstants.extensor.gearing)
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("ExtensorMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController extensorMotor =
      new SparkWrapper(extensorSparkFlex, IntakeConstants.extensor.motor, extensorMotorConfig);

  private final ElevatorConfig extensorConfig =
      new ElevatorConfig(extensorMotor)
          .withMass(IntakeConstants.extensor.mass)
          .withHardLimits(
              IntakeConstants.limits.MIN_EXTENSION, IntakeConstants.limits.MAX_EXTENSION)
          .withTelemetry("ExtensorMech", TelemetryVerbosity.HIGH)
          .withMechanismPositionConfig(IntakeConstants.extensor.robotToMechanism);

  private final Elevator extensor = new Elevator(extensorConfig);

  public ExtensorSubsystem() {}

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Subsystems/Intake", extensorInputs);
    extensor.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    extensor.simIterate();
  }

  public void updateInputs() {
    extensorInputs.extensorVoltage = extensorMotor.getVoltage();
    extensorInputs.extensorCurrent = extensorMotor.getStatorCurrent();
    extensorInputs.extensorPosition = extensorMotor.getMeasurementPosition();
    extensorInputs.extensorVelocity = extensorMotor.getMeasurementVelocity();
  }
}
