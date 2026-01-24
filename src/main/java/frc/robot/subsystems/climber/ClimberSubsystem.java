package frc.robot.subsystems.climber;

import frc.java_is_undercontrol.motors.MotorIO;
import frc.java_is_undercontrol.motors.MotorIOInputsAutoLogged;
import frc.java_is_undercontrol.motors.SparkFlexMotor;
import frc.robot.constants.climber.ClimberConstants;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem {
  private static ClimberSubsystem instance;
  private final MotorIOInputsAutoLogged climberInputs;
  private final MotorIO climberMotor;

  public static ClimberSubsystem getInstance() {
    if (instance == null) {
      instance = new ClimberSubsystem();
    }
    return instance;
  }

  public ClimberSubsystem() {
    this.climberInputs = new MotorIOInputsAutoLogged();
    this.climberMotor = new SparkFlexMotor(ClimberConstants.ID_climberMotor, "ClimberMotor");
    this.configureClimberMotor();
  }

  private void configureClimberMotor() {
    climberMotor.setInverted(false);
    climberMotor.setPositionFactor(250);
    climberMotor.configurePIDF(
        ClimberConstants.PID.P,
        ClimberConstants.PID.I,
        ClimberConstants.PID.D,
        ClimberConstants.PID.F);
    climberMotor.burnFlash();
    climberMotor.setMotorBrake(true);
    climberMotor.setPosition(0);
  }

  public void periodic() {
    this.updateLogs();
  }

  private void updateLogs() {
    this.climberMotor.updateInputs(climberInputs);
    this.climberMotor.updateInputs(climberInputs);

    Logger.processInputs("Motors/Climber", climberInputs);
  }

  public void setCoastClimber() {
    climberMotor.setMotorBrake(false);
  }

  public void prepareToClimb() {
    climberMotor.setPosition(ClimberConstants.setpoints.PREPARE_TO_CLIMB);
  }

  public void climb() {
    climberMotor.setPosition(ClimberConstants.setpoints.CLIMB_ANGLE);
  }

  public void releaseClimber() {}
}
