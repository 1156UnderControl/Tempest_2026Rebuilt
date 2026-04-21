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

  private final SparkFlex rightSparkFlex;
  private final SparkFlex leftSparkFlex;

  private final SparkFlexConfig rightConfig;
  private final SparkFlexConfig leftConfig;

  private RollerSubsystem() {
    this.rollerInputs = new RollerInputsAutoLogged();

    this.rightSparkFlex =
        new SparkFlex(RollerConstants.ID_intakeRightRollerMotor, MotorType.kBrushless);
    this.leftSparkFlex =
        new SparkFlex(RollerConstants.ID_intakeLeftRollerMotor, MotorType.kBrushless);

    this.rightConfig = new SparkFlexConfig();
    this.rightConfig.idleMode(IdleMode.kCoast).inverted(true).smartCurrentLimit(70);
    this.rightSparkFlex.configure(
        rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    this.leftConfig = new SparkFlexConfig();
    this.leftConfig.idleMode(IdleMode.kCoast).inverted(false).smartCurrentLimit(70);
    this.leftSparkFlex.configure(
        leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Subsystems/Roller", rollerInputs);
  }

  public void updateInputs() {
    rollerInputs.rollerRightAppliedVolts = Volts.of(rightSparkFlex.getAppliedOutput() * 12);
    rollerInputs.rollerLeftAppliedVolts = Volts.of(leftSparkFlex.getAppliedOutput() * 12);
    rollerInputs.rollerRightCurrent = Amps.of(rightSparkFlex.getOutputCurrent());
    rollerInputs.rollerLeftCurrent = Amps.of(leftSparkFlex.getOutputCurrent());
    rollerInputs.rollerRightVelocity =
        RPM.of(rightSparkFlex.getAbsoluteEncoder().getVelocity() / RollerConstants.GEARING);
    rollerInputs.rollerLeftVelocity =
        RPM.of(leftSparkFlex.getAbsoluteEncoder().getVelocity() / RollerConstants.GEARING);
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
          this.rightSparkFlex.setVoltage(voltage);
          this.leftSparkFlex.setVoltage(voltage);
        });
  }
}
