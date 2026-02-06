package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.intake.RollerConstants;
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

  private final SparkFlex rollerMasterFlex;
  private final SparkFlex rollerFollowerFlex;

  private SmartMotorControllerConfig smartRollerMotorConfig;

  private final SmartMotorController rollerMotor;

  private final FlyWheelConfig rollerConfig;

  private final FlyWheel roller;

  private AngularVelocity setpoint = RPM.of(0);

  public RollerSubsystem() {
    this.rollerMasterFlex = new SparkFlex(RollerConstants.ID_MOTOR_LEADER, MotorType.kBrushless);
    this.rollerFollowerFlex =
        new SparkFlex(RollerConstants.ID_MOTOR_FOLLOWER, MotorType.kBrushless);

    this.smartRollerMotorConfig =
        new SmartMotorControllerConfig(this)
            .withMechanismCircumference(Inches.of(4))
            .withFollowers(Pair.of(rollerFollowerFlex, true))
            .withClosedLoopController(0, 0, 0, RPM.of(3000), RotationsPerSecondPerSecond.of(150))
            .withSimClosedLoopController(
                50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(2)))
            .withIdleMode(MotorMode.COAST)
            .withTelemetry("RollerMotor", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(false)
            .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25));

    this.rollerMotor =
        new SparkWrapper(
            this.rollerMasterFlex, DCMotor.getNeoVortex(2), this.smartRollerMotorConfig);

    this.rollerConfig =
        new FlyWheelConfig(rollerMotor)
            // Diameter of the flywheel.
            .withDiameter(Inches.of(4))
            // Mass of the flywheel.
            .withMass(Pounds.of(1))
            .withMOI(KilogramSquareMeters.of(0.003))
            .withTelemetry("RollerMech", TelemetryVerbosity.HIGH);

    this.roller = new FlyWheel(rollerConfig);
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Subsystems/Roller", rollerInputs);
    roller.updateTelemetry();
    this.setpoint =
        rollerMotor.getMechanismSetpointVelocity().isEmpty()
            ? RPM.of(0)
            : rollerMotor.getMechanismSetpointVelocity().get();
  }

  @Override
  public void simulationPeriodic() {
    this.roller.simIterate();
  }

  public void updateInputs() {
    rollerInputs.rollerVoltage = rollerMotor.getVoltage();
    rollerInputs.rollerCurrent = rollerMotor.getStatorCurrent();
    rollerInputs.rollerPosition = rollerMotor.getMechanismPosition();
    rollerInputs.rollerVelocity = rollerMotor.getMechanismVelocity();
  }

  public Command runRoller() {
    return this.setDutyCycle(RollerConstants.setpoints.INTAKE_DUTY_CYCLE);
  }

  public Command runRollerReversed() {
    return this.setDutyCycle(RollerConstants.setpoints.REVERSE_DUTY_CYCLE);
  }

  public Command stopRoller() {
    return this.setDutyCycle(RollerConstants.setpoints.STOP_DUTY_CYCLE);
  }

  private Command setVoltage(Voltage voltage) {
    return roller.setVoltage(voltage);
  }

  private Command setDutyCycle(double dutycycle) {
    return roller.set(dutycycle);
  }

  public Trigger isAtVelocity() {
    return roller.isNear(setpoint, RollerConstants.VELOCITY_ERROR_ALOWED);
  }
}
