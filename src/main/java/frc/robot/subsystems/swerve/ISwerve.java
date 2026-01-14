package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface ISwerve {

  void resetOdometryLimelight(Translation2d defaultPosition);

  void driveAlignAngleJoystick();

  void driveAlignAngleJoystickSuperSlow();

  void stopSwerve();

  boolean isAtTargetPositionWithHeading();

  boolean isAtTargetPositionWithoutHeading();

  void driveAimingAtPoseTest();

  Command runSysIdQuasistaticForward();

  Command runSysIdQuasistaticReversed();

  Command runSysIdDynamicForward();

  Command runSysIdDynamicReversed();
}
