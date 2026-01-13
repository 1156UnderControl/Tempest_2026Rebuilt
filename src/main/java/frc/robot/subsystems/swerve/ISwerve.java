package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;

public interface ISwerve {

  void resetOdometryLimelight(Translation2d defaultPosition);

  void driveAlignAngleJoystick();

  void driveAlignAngleJoystickSuperSlow();

  void stopSwerve();

  boolean isAtTargetPositionWithHeading();

  boolean isAtTargetPositionWithoutHeading();

  void driveAimingAtPoseTest();
}
