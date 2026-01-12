package frc.java_is_undercontrol.vision.deprecated.ObjectDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.java_is_undercontrol.util.CoordinatesTransform;

public class ObjectPoseEstimationRobotOriented {
  ObjectDetection object;

  ObjectDetectionCamera camera;

  double angleObjectInMaxTy;

  double angleObjectInMaxTx;

  public ObjectPoseEstimationRobotOriented(
      ObjectDetection object,
      ObjectDetectionCamera camera,
      double angleObjectInMaxTy,
      double angleObjectInMaxTx) {
    this.object = object;
    this.camera = camera;
    this.angleObjectInMaxTy = angleObjectInMaxTy;
    this.angleObjectInMaxTx = angleObjectInMaxTx;
  }

  public Pose2d getObjectPoseInField(Pose2d robotPose, Pose2d objectPose) {
    Pose2d poseInTheField2d = CoordinatesTransform.fromRobotRelativeToF(robotPose, objectPose);
    Pose3d poseInTheField3d = new Pose3d(poseInTheField2d);
    return poseInTheField2d;
  }

  public Pose2d getObjectPoseFromRobotCenter() {
    return new Pose2d(
        getXnoteDistanceFromRobotCenter(), -getYnoteDistanceFromRobotCenter(), new Rotation2d());
  }

  public double getXnoteDistanceFromRobotCenter() {
    double calculation =
        (getDistanceFromObject()
                * Math.cos(
                    Units.radiansToDegrees(camera.robotToCamera.getRotation().getZ())
                        + Units.degreesToRadians(getObjectAngleX())))
            + camera.robotToCamera.getX();
    return calculation;
  }

  public double getYnoteDistanceFromRobotCenter() {
    double calculation =
        (getDistanceFromObject()
                * Math.sin(
                    Units.radiansToDegrees(camera.robotToCamera.getRotation().getZ())
                        + Units.degreesToRadians(getObjectAngleX())))
            + camera.robotToCamera.getY();
    return calculation;
  }

  public double getDistanceFromObject() {
    double calculation =
        camera.cameraHeight
            * Math.tan(
                Units.degreesToRadians(getObjectAngleY())
                    + Units.degreesToRadians(camera.getCameraAngle()));
    return calculation;
  }

  private double getObjectAngleY() {
    double calculation = (object.ty * angleObjectInMaxTy) / camera.maxTy;
    return calculation;
  }

  private double getObjectAngleX() {
    double calculation = (object.tx * angleObjectInMaxTx) / camera.maxTx;
    return calculation;
  }
}
