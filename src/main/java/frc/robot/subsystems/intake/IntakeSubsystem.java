package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.java_is_undercontrol.motors.MotorIO;
import frc.java_is_undercontrol.motors.SparkFlexMotor;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final MotorIO intakeWheels;
  private final MotorIO intakeExtender;

  public IntakeSubsystem() {
    intakeExtender =
        new SparkFlexMotor(IntakeConstants.ID_intakeExtenderMotor, "Intake Extender Motor");
    intakeWheels = new SparkFlexMotor(IntakeConstants.ID_intakeWheelsMotor, "Intake Wheels Motor");
  }

  private void setIntakeExtenderConfigs() {
    intakeExtender.setMotorBrake(true);
    intakeExtender.configurePIDF(
        IntakeConstants.PID.P, IntakeConstants.PID.I, IntakeConstants.PID.D, IntakeConstants.PID.F);
    intakeExtender.burnFlash();
  }

  public void collectFuel() {
    intakeWheels.set(0.8);
  }

  public void goToCollectPostion() {
    intakeExtender.setPositionReference(IntakeConstants.setpoints.INTAKE_EXTENDED_POSITION);
  }

  public void goToHomedPosition() {
    intakeExtender.setPositionReference(IntakeConstants.setpoints.INTAKE_HOMED_POSITION);
  }

  public void stopIntake() {
    intakeWheels.set(IntakeConstants.motor_values.INTAKE_WHEELS_STOP);
  }

  @Override
  public void periodic() {}
}
