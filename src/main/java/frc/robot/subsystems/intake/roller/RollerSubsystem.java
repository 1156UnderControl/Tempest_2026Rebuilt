package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class RollerSubsystem extends SubsystemBase {

  private static RollerSubsystem instance;

  public static RollerSubsystem getInstance() {
    if (instance == null) {
      instance = new RollerSubsystem();
    }
    return instance;
  }

  private final RollerInputsAutoLogged rollerInputs;

  private final TalonFX leadMotor;
  private final TalonFX followerMotor;

  private final VoltageOut leadRequest = new VoltageOut(0).withEnableFOC(true);
  private final VoltageOut followerRequest = new VoltageOut(0).withEnableFOC(true);

  private Current statorLimit = RollerConstants.STATOR_LIMIT_TELEOP;
  private Current supplyLimit = RollerConstants.SUPPLY_LIMIT_TELEOP;

  private RollerSubsystem() {
    this.rollerInputs = new RollerInputsAutoLogged();

    this.leadMotor = new TalonFX(RollerConstants.ID_intakeLeaderRollerMotor);
    this.followerMotor = new TalonFX(RollerConstants.ID_intakeFollowerRollerMotor);

    TalonFXConfiguration leadConfig = new TalonFXConfiguration();
    leadConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leadConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leadConfig.CurrentLimits
        .withStatorCurrentLimit(statorLimit)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(supplyLimit)
        .withSupplyCurrentLimitEnable(true);
    leadMotor.getConfigurator().apply(leadConfig);

    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    followerConfig.CurrentLimits
        .withStatorCurrentLimit(statorLimit)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(supplyLimit)
        .withSupplyCurrentLimitEnable(true);
    followerMotor.getConfigurator().apply(followerConfig);
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Subsystems/Roller", rollerInputs);
  }

  public void updateInputs() {
    rollerInputs.rollerLeadAppliedVolts = Volts.of(leadMotor.getMotorVoltage().getValueAsDouble());
    rollerInputs.rollerFollowerAppliedVolts =
        Volts.of(followerMotor.getMotorVoltage().getValueAsDouble());
    rollerInputs.rollerLeadCurrent = Amps.of(leadMotor.getStatorCurrent().getValueAsDouble());
    rollerInputs.rollerFollowerCurrent =
        Amps.of(followerMotor.getStatorCurrent().getValueAsDouble());
    rollerInputs.rollerLeadVelocity =
        RPM.of(leadMotor.getVelocity().getValueAsDouble() * 60.0 / RollerConstants.GEARING);
    rollerInputs.rollerFollowerVelocity =
        RPM.of(followerMotor.getVelocity().getValueAsDouble() * 60.0 / RollerConstants.GEARING);
  }

  public Command runRoller() {
    return setVoltage(Volts.of(12.0));
  }

  public Command expellRoller() {
    return setVoltage(Volts.of(-12.0));
  }

  public Command stopRoller() {
    return setVoltage(Volts.of(0));
  }

  private Command setVoltage(Voltage voltage) {
    return this.run(
        () -> {
          leadMotor.setControl(leadRequest.withOutput(voltage));
          followerMotor.setControl(followerRequest.withOutput(voltage));
        });
  }

  private void applyCurrentLimits() {
    CurrentLimitsConfigs limits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(statorLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(supplyLimit)
            .withSupplyCurrentLimitEnable(true);
    leadMotor.getConfigurator().apply(limits);
    followerMotor.getConfigurator().apply(limits);
  }

  public void configureTeleop() {
    statorLimit = RollerConstants.STATOR_LIMIT_TELEOP;
    supplyLimit = RollerConstants.SUPPLY_LIMIT_TELEOP;
    applyCurrentLimits();
  }

  public void configureAuto() {
    statorLimit = RollerConstants.STATOR_LIMIT_AUTO;
    supplyLimit = RollerConstants.SUPPLY_LIMIT_AUTO;
    applyCurrentLimits();
  }

  public void configureShooting() {
    statorLimit = RollerConstants.STATOR_LIMIT_SHOOTING;
    supplyLimit = RollerConstants.SUPPLY_LIMIT_SHOOTING;
    applyCurrentLimits();
  }
}
