package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
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

  private final SparkFlex leadSparkFlex;
  private final SparkFlex followerSparkFlex;

  private final SparkFlexConfig leadConfig;
  private final SparkFlexConfig followerConfig;

  private RollerSubsystem() {
    this.rollerInputs = new RollerInputsAutoLogged();

    this.leadSparkFlex =
        new SparkFlex(RollerConstants.ID_intakeLeaderRollerMotor, MotorType.kBrushless);
    this.followerSparkFlex =
        new SparkFlex(RollerConstants.ID_intakeFollowerRollerMotor, MotorType.kBrushless);

    this.leadConfig = new SparkFlexConfig();
    this.leadConfig.idleMode(IdleMode.kCoast).inverted(true).smartCurrentLimit(70);
    this.leadSparkFlex.configure(
        leadConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    this.followerConfig = new SparkFlexConfig();
    this.followerConfig.idleMode(IdleMode.kCoast).inverted(false).smartCurrentLimit(70);
    this.followerSparkFlex.configure(
        followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Subsystems/Roller", rollerInputs);
  }

  public void updateInputs() {
    rollerInputs.rollerLeadAppliedVolts = Volts.of(leadSparkFlex.getAppliedOutput() * 12);
    rollerInputs.rollerFollowerAppliedVolts = Volts.of(followerSparkFlex.getAppliedOutput() * 12);
    rollerInputs.rollerLeadCurrent = Amps.of(leadSparkFlex.getOutputCurrent());
    rollerInputs.rollerFollowerCurrent = Amps.of(followerSparkFlex.getOutputCurrent());
    rollerInputs.rollerLeadVelocity =
        RPM.of(leadSparkFlex.getAbsoluteEncoder().getVelocity() / RollerConstants.GEARING);
    rollerInputs.rollerFollowerVelocity =
        RPM.of(followerSparkFlex.getAbsoluteEncoder().getVelocity() / RollerConstants.GEARING);
  }

  public Command runRoller() {
    return this.setVoltage(Volts.of(12.0));
  }

  public Command expellRoller() {
    return this.setVoltage(Volts.of(-12.0));
  }

  public Command stopRoller() {
    return this.setVoltage(Volts.of(0));
  }

  private Command setVoltage(Voltage voltage) {
    return this.run(
        () -> {
          this.leadSparkFlex.setVoltage(voltage);
          this.followerSparkFlex.setVoltage(voltage);
        });
  }
}
