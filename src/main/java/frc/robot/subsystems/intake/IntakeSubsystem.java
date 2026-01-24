package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.java_is_undercontrol.motors.MotorIO;
import frc.java_is_undercontrol.motors.SparkFlexMotor;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final MotorIO intakeWheels;
  private final MotorIO intakeExtender;
  private final MotorIO spindexerMotor;

  public IntakeSubsystem() {
    intakeExtender =
        new SparkFlexMotor(IntakeConstants.ID_intakeExtenderMotor, "Intake Extender Motor");
    intakeWheels = new SparkFlexMotor(IntakeConstants.ID_intakeWheelsMotor, "Intake Wheels Motor");
    spindexerMotor = new SparkFlexMotor(IntakeConstants.ID_spindexerMotor, "Spindxer Motor");
  }

  private void setIntakeExtenderConfigs() {
    intakeExtender.setMotorBrake(true);
    intakeExtender.configurePIDF(
        IntakeConstants.PID.P, IntakeConstants.PID.I, IntakeConstants.PID.D, IntakeConstants.PID.F);
    intakeExtender.burnFlash();
  }

  public void collectFuel() {
    intakeWheels.set(0.6);
  }

  public void stopCollectingFuel() {
    intakeWheels.set(IntakeConstants.motor_values.INTAKE_WHEELS_STOP);
  }

  public void goToCollectPostion() {
    intakeExtender.setPositionReference(IntakeConstants.setpoints.INTAKE_EXTENDED_POSITION);
  }

  public void goToHomedPosition() {
    intakeExtender.setPositionReference(IntakeConstants.setpoints.INTAKE_HOMED_POSITION);
  }

  public void runSpindexer() {
    spindexerMotor.set(0.6);
  }

  public void stopSpindexer() {
    spindexerMotor.set(0);
  }

  public void agigateIntake() {}

  @Override
  public void periodic() {}
}
