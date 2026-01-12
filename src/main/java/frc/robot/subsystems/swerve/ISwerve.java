package frc.robot.subsystems.swerve;

public interface ISwerve {

  void driveAlignAngleJoystick();

  void driveAlignAngleJoystickSuperSlow();

  void stopSwerve();

  boolean isAtTargetPositionWithHeading();

  boolean isAtTargetPositionWithoutHeading();
}
