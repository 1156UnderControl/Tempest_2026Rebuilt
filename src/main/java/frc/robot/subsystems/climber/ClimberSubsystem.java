package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.Timer;
import frc.java_is_undercontrol.motors.MotorIO;
import frc.java_is_undercontrol.motors.MotorIOInputsAutoLogged;
import frc.java_is_undercontrol.motors.SparkFlexMotor;
import frc.robot.constants.ClimberConstants;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem implements IClimber{
private static ClimberSubsystem instance;
private final MotorIOInputsAutoLogged climberInputs;
private final MotorIO climberMotor;
private final Timer timer;




public static ClimberSubsystem getInstance(){
    if (instance == null) {
        instance = new ClimberSubsystem();
    }
    return instance;


}

public ClimberSubsystem(){
    this.climberInputs = new MotorIOInputsAutoLogged();
    this.climberMotor = new SparkFlexMotor(ClimberConstants.ID_climberMotor, 1, "ClimberMotor"); 
    this.timer = new Timer();
    this.configureClimberMotor();
    
}

 private void configureClimberMotor() {
        climberMotor.setInverted(false);
        climberMotor.setPositionFactor(250);
        climberMotor.configurePIDF(
            ClimberConstants.PID.P,
            ClimberConstants.PID.I,
            ClimberConstants.PID.D, 0);
        climberMotor.burnFlash();
        climberMotor.setMotorBrake(true);
        climberMotor.setPosition(0);
    }

    
    public void periodic(){
        this.updateLogs();
    }

    private void updateLogs() {
        this.climberMotor.updateInputs(climberInputs);
        this.climberMotor.updateInputs(climberInputs);

        Logger.processInputs("Motors/Climber", climberInputs);
    }
    


    public void startCollecting(){
    climberMotor.set(-1);
    }

    public void setBrakeClimber(){
        climberMotor.setMotorBrake(true);
    }

     public void setCoastClimber(){
        climberMotor.setMotorBrake(false);
    }

    public void Climb(){
        climberMotor.setPosition(ClimberConstants.setpoints.CLIMB_ANGLE);
    }

   
    public boolean isClimbed(){
        climberMotor.setPosition(ClimberConstants.setpoints.CLIMBED_ANGLE);
        return true;
    }

    public void defaultPosition(){
         climberMotor.setPosition(ClimberConstants.setpoints.DEFAULT_ANGLE);
    }


}

