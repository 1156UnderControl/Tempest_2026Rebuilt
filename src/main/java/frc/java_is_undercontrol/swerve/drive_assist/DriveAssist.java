package frc.java_is_undercontrol.swerve.drive_assist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.java_is_undercontrol.util.CoordinatesTransform;

public class DriveAssist {
  Translation2d correctionReferencePosition;
  double correntionPercentage;
  double maxDistanceToCorrect;
  double maxJoystickAngleToCorrect;

  public DriveAssist() {
    this(new Translation2d(), 1, 2, 45);
  }

  public DriveAssist(
      Translation2d correctionReferencePosition,
      double correctionPercentage,
      double maxDistanceToCorrect,
      double maxJoystickAngleToCorrect) {
    this.correctionReferencePosition = correctionReferencePosition;
    this.correntionPercentage = correctionPercentage;
    this.maxDistanceToCorrect = maxDistanceToCorrect;
    this.maxJoystickAngleToCorrect = maxJoystickAngleToCorrect;
  }

  public ChassisSpeeds getCorrectedDesiredSpeedToTranslation(
      Pose2d robotPose, Pose2d objectPose, ChassisSpeeds robotVelocity) {
    if (robotPose.getTranslation().getDistance(objectPose.getTranslation())
        > this.maxDistanceToCorrect) {
      return robotVelocity;
    }
    Translation2d correctedReferencePose =
        CoordinatesTransform.fromRobotRelativeToF(
                robotPose, new Pose2d(correctionReferencePosition, new Rotation2d()))
            .getTranslation();
    Translation2d perfectTranslationDirection =
        objectPose.getTranslation().minus(correctedReferencePose);
    Rotation2d perfectTranslationAngle = perfectTranslationDirection.getAngle();
    Rotation2d requestedTranslationAngle =
        new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond)
            .getAngle();
    Rotation2d requestedToPerfectAngleDifference =
        perfectTranslationAngle.minus(requestedTranslationAngle);
    if (Math.abs(requestedToPerfectAngleDifference.getDegrees()) > this.maxJoystickAngleToCorrect) {
      return robotVelocity;
    }
    Rotation2d desiredTranslationAngle =
        requestedTranslationAngle.plus(
            requestedToPerfectAngleDifference.times(correntionPercentage));
    double robotVelocityIntensity =
        Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);
    double desiredVx = robotVelocityIntensity * desiredTranslationAngle.getCos();
    double desiredVy = robotVelocityIntensity * desiredTranslationAngle.getSin();
    return new ChassisSpeeds(desiredVx, desiredVy, robotVelocity.omegaRadiansPerSecond);
  }
}
