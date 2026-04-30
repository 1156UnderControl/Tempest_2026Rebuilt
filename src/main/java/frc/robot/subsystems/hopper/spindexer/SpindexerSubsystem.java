package frc.robot.subsystems.hopper.spindexer;

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

public class SpindexerSubsystem extends SubsystemBase {
  private static SpindexerSubsystem instance;

  public static SpindexerSubsystem getInstance() {
    if (instance == null) {
      instance = new SpindexerSubsystem();
    }
    return instance;
  }

  private final SpindexerInputsAutoLogged spindexerInputs = new SpindexerInputsAutoLogged();

  private final SparkFlex leadSparkFlex;
  private final SparkFlex followSparkFlex;

  private final SparkFlexConfig leadConfig;
  private final SparkFlexConfig followerConfig;

  private SpindexerSubsystem() {
    this.leadSparkFlex = new SparkFlex(SpindexerConstants.LEADER_MOTOR_ID, MotorType.kBrushless);
    this.followSparkFlex = new SparkFlex(SpindexerConstants.FOLLOW_MOTOR_ID, MotorType.kBrushless);

    this.leadConfig = new SparkFlexConfig();
    this.followerConfig = new SparkFlexConfig();

    this.leadConfig.idleMode(IdleMode.kCoast).inverted(false).smartCurrentLimit(55);
    this.leadSparkFlex.configure(
        leadConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    leadConfig.openLoopRampRate(0.01);

    this.followerConfig.idleMode(IdleMode.kCoast).inverted(true).smartCurrentLimit(55);
    this.followSparkFlex.configure(
        followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    followerConfig.openLoopRampRate(0.01);
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Subsystems/Spindexer", spindexerInputs);
  }

  public void updateInputs() {
    spindexerInputs.spindexerLeadAppliedVolts = Volts.of(leadSparkFlex.getAppliedOutput() * 12);
    spindexerInputs.spindexerFollowerAppliedVolts =
        Volts.of(followSparkFlex.getAppliedOutput() * 12);
    spindexerInputs.spindexerLeadCurrent = Amps.of(leadSparkFlex.getOutputCurrent());
    spindexerInputs.spindexerFollowerCurrent = Amps.of(followSparkFlex.getOutputCurrent());
    spindexerInputs.spindexerLeadVelocity =
        RPM.of(leadSparkFlex.getAbsoluteEncoder().getVelocity() / SpindexerConstants.GEARING);
    spindexerInputs.spindexerFollowerVelocity =
        RPM.of(followSparkFlex.getAbsoluteEncoder().getVelocity() / SpindexerConstants.GEARING);
  }

  public Command shoot() {
    return this.setVoltage(Volts.of(12.0));
  }

  public Command expell() {
    return this.setVoltage(Volts.of(-6.0));
  }

  public Command stop() {
    return this.setVoltage(Volts.of(0));
  }

  public Command setVoltage(Voltage voltage) {
    return this.run(
        () -> {
          leadSparkFlex.setVoltage(voltage);
          followSparkFlex.setVoltage(voltage);
        });
  }
}
