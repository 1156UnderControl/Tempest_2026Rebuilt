package frc.robot.subsystems.hopper.spindexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
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

  private final TalonFX spindexerTalonFX;



  private SpindexerSubsystem() {
    this.spindexerTalonFX = new TalonFX(SpindexerConstants.LEADER_MOTOR_ID);
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Subsystems/Spindexer", spindexerInputs);
  }

  public void updateInputs() {

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
          spindexerTalonFX.setVoltage(-12);
        });
  }
}
