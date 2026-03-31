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

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

  private static RollerSubsystem instance;

  public static RollerSubsystem getInstance() {
    if (instance == null) {
      instance = new RollerSubsystem();
    }
    return instance;
  }

  private final RollerInputsAutoLogged rollerInputs;

  private final SparkFlex rollerMasterFlex;
  private final SparkFlex rollerFollowerFlex;

  private SparkFlexConfig followerConfig;

  private SmartMotorControllerConfig smartRollerMotorConfig;

  private final SmartMotorController rollerMotor;

  private final FlyWheelConfig rollerConfig;

  private final FlyWheel roller;

  private AngularVelocity setpoint = RPM.of(0);

  private RollerSubsystem() {
    this.rollerInputs = new RollerInputsAutoLogged();
    this.rollerMasterFlex =
        new SparkFlex(RollerConstants.ID_intakeLeaderRollerMotor, MotorType.kBrushless);
    this.rollerFollowerFlex =
        new SparkFlex(RollerConstants.ID_intakeFollowerRollerMotor, MotorType.kBrushless);

    this.followerConfig = new SparkFlexConfig();
    this.followerConfig.idleMode(IdleMode.kCoast);
    this.rollerFollowerFlex.configure(
        followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    this.smartRollerMotorConfig =
        new SmartMotorControllerConfig(this)
            .withMechanismCircumference(RollerConstants.MECHANISM_CIRCUNFERENCE)
            .withFollowers(Pair.of(rollerFollowerFlex, true))
            .withClosedLoopController(0, 0, 0, RPM.of(3000), RotationsPerSecondPerSecond.of(150))
            .withSimClosedLoopController(
                50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(2)))
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(60))
            .withMotorInverted(false)
            .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withControlMode(ControlMode.OPEN_LOOP)
            .withTelemetry(TelemetryVerbosity.MID)
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
            .withMOI(KilogramSquareMeters.of(0.003));

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
    return this.setDutyCycle(-1.0);
  }

  public Command expellRoller() {
    return this.setDutyCycle(1.0);
  }

  public Command stopRoller() {
    return this.setDutyCycle(RollerConstants.setpoints.STOP_DUTY_CYCLE);
  }

  private Command setDutyCycle(double dutycycle) {
    return roller.set(dutycycle);
  }

  public Trigger isAtVelocity() {
    return roller.isNear(setpoint, RollerConstants.VELOCITY_ERROR_ALOWED);
  }
}
