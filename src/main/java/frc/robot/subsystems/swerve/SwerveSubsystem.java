package frc.robot.subsystems.swerve;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.java_is_undercontrol.control.ProfiledPIDConfig;
import frc.java_is_undercontrol.swerve.SwerveDevices;
import frc.java_is_undercontrol.swerve.base.odometry.OdometryEnabledSwerveSubsystem;
import frc.java_is_undercontrol.swerve.configs.MoveToPosePIDConfig;
import frc.java_is_undercontrol.swerve.configs.OdometryEnabledSwerveConfig;
import frc.java_is_undercontrol.swerve.configs.SwerveDeadbandsConfig;
import frc.java_is_undercontrol.swerve.configs.SwervePathPlannerConfig;
import frc.java_is_undercontrol.util.StabilizeChecker;
import frc.java_is_undercontrol.vision.Cameras.LimelightHelpers;
import frc.java_is_undercontrol.vision.odometry.LimelightPoseEstimator;
import frc.java_is_undercontrol.vision.odometry.MultiCameraPoseEstimator;
import frc.java_is_undercontrol.vision.odometry.PhotonVisionPoseEstimator;
import frc.java_is_undercontrol.vision.odometry.PoseEstimation;
import frc.java_is_undercontrol.vision.odometry.PoseEstimator;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.JoystickConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.joysticks.DriverController;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.PhotonCamera;

public class SwerveSubsystem extends OdometryEnabledSwerveSubsystem {

  private DriverController controller = DriverController.getInstance();

  private Matrix<N3, N1> defaultVisionStandardDeviation;

  private Matrix<N3, N1> defaultOdometryStandardDeviation;

  private double goToPoseTranslationDeadband = 0.03;

  private double goToPoseHeadingDeadband = 1;

  int[] apriltagsIDs =
      new int[] {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};

  private StabilizeChecker stableAtTargetPose = new StabilizeChecker(0.150);

  @AutoLogOutput String state = "NULL";

  @AutoLogOutput String poseEstimator;

  @AutoLogOutput boolean isUsingAngleCorrection;

  @AutoLogOutput double targetVelocity;

  @AutoLogOutput double distanceToTargetBranchLog;

  @AutoLogOutput double distanceToTargetFaceLog;

  @AutoLogOutput private boolean positionUpdated = false;

  private final Supplier<Pose2d> robotPose;

  private final Supplier<ChassisSpeeds> currentSpeeds;

  private static final SwervePathPlannerConfig pathPlannerConfig =
      new SwervePathPlannerConfig(
          new PIDConstants(5, 0, 0),
          new PIDConstants(5, 0, 0),
          new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));

  public SwerveSubsystem(SwerveDevices swerveDevices) {
    super(
        new OdometryEnabledSwerveConfig(
            0.75,
            0.5,
            new SwerveDeadbandsConfig(
                JoystickConstants.DEADBAND_TRANSLATION, JoystickConstants.DEADBAND_ROTATION),
            pathPlannerConfig,
            configureMulticameraPoseEstimation(),
            configureMulticameraPoseEstimation(),
            new ProfiledPIDConfig(7, 0, 0.1, 80, 50),
            new MoveToPosePIDConfig(
                SwerveConstants.MOVE_TO_POSE_TRANSLATION_PID,
                SwerveConstants.MOVE_TO_POSE_Y_CONSTRAINTS)),
        swerveDevices);
    this.resetOdometry(new Pose2d());
    this.robotPose = () -> super.getPose();
    this.currentSpeeds = () -> super.getRobotVelocity();
  }

  private static PoseEstimator configureMulticameraPoseEstimation() {
    List<PoseEstimator> listOfEstimators = new ArrayList<PoseEstimator>();
    PoseEstimator limelightBackLeft =
        new LimelightPoseEstimator(VisionConstants.backLeftLimelightName, false, false, 2);
    PoseEstimator limelightBackRight =
        new LimelightPoseEstimator(VisionConstants.backRightLimelightName, false, false, 2);
    PhotonCamera cameraFrontRight = new PhotonCamera(VisionConstants.frontRightArducamName);
    PhotonCamera cameraFrontLeft = new PhotonCamera(VisionConstants.frontLeftArducamName);

    PoseEstimator photonFrontRight =
        new PhotonVisionPoseEstimator(
            cameraFrontRight, VisionConstants.robotToCamArducamFrontRight);

    PoseEstimator photonFrontLeft =
        new PhotonVisionPoseEstimator(cameraFrontLeft, VisionConstants.robotToCamArducamFrontLeft);

    listOfEstimators.add(limelightBackLeft);
    listOfEstimators.add(limelightBackRight);
    listOfEstimators.add(photonFrontRight);
    listOfEstimators.add(photonFrontLeft);

    PoseEstimator estimatorMultiCamera =
        new MultiCameraPoseEstimator(listOfEstimators, "Teleop Multi Pose Estimator");
    return estimatorMultiCamera;
  }

  public void resetOdometryLimelight(Translation2d defaultPosition) {
    PoseEstimator limelightBackLeft =
        new LimelightPoseEstimator(VisionConstants.backLeftLimelightName, false, false, 2);
    PoseEstimator limelightBackRight =
        new LimelightPoseEstimator(VisionConstants.backRightLimelightName, false, false, 2);

    Optional<PoseEstimation> limelightBackLeftPoseEstimation =
        limelightBackLeft.getEstimatedPose(this.getPose());

    if (limelightBackLeftPoseEstimation.isEmpty()) {
      resetTranslation(defaultPosition);
      this.positionUpdated = true;
    } else {
      resetTranslation(
          limelightBackLeftPoseEstimation.get().estimatedPose.getTranslation().toTranslation2d());
      this.positionUpdated = true;
    }

    Optional<PoseEstimation> limelightBackRightPoseEstimation =
        limelightBackRight.getEstimatedPose(this.getPose());
    if (limelightBackRightPoseEstimation.isEmpty()) {
      resetTranslation(defaultPosition);
      this.positionUpdated = true;
    } else {
      resetTranslation(
          limelightBackRightPoseEstimation.get().estimatedPose.getTranslation().toTranslation2d());
      this.positionUpdated = true;
    }
  }

  public void resetOdometryPhotonvision(Translation2d defaultPosition) {
    PhotonCamera cameraFrontRight = new PhotonCamera(VisionConstants.frontRightArducamName);
    PoseEstimator photonFrontRight =
        new PhotonVisionPoseEstimator(
            cameraFrontRight, VisionConstants.robotToCamArducamFrontRight);

    PhotonCamera cameraFrontLeft = new PhotonCamera(VisionConstants.frontLeftArducamName);
    PoseEstimator photonFrontLeft =
        new PhotonVisionPoseEstimator(cameraFrontLeft, VisionConstants.robotToCamArducamFrontLeft);

    Optional<PoseEstimation> photonFrontRightPoseEstimation =
        photonFrontRight.getEstimatedPose(this.getPose());

    if (photonFrontRightPoseEstimation.isEmpty()) {
      resetTranslation(defaultPosition);
      this.positionUpdated = true;
    } else {
      resetTranslation(
          photonFrontRightPoseEstimation.get().estimatedPose.getTranslation().toTranslation2d());
      this.positionUpdated = true;
    }

    Optional<PoseEstimation> photonFrontLeftPoseEstimation =
        photonFrontLeft.getEstimatedPose(this.getPose());

    if (photonFrontLeftPoseEstimation.isEmpty()) {
      resetTranslation(defaultPosition);
      this.positionUpdated = true;
    } else {
      resetTranslation(
          photonFrontLeftPoseEstimation.get().estimatedPose.getTranslation().toTranslation2d());
      this.positionUpdated = true;
    }
  }

  public Supplier<Pose2d> getOdometryPose() {
    return this.robotPose;
  }

  public Supplier<ChassisSpeeds> getCurrentSpeeds() {
    return this.currentSpeeds;
  }

  public void driveAlignAngleJoystick() {
    if (controller.leftBumper().getAsBoolean()) {
      driveRotating(false);
      return;
    }
    if (controller.rightBumper().getAsBoolean()) {
      driveRotating(true);
      return;
    }
    Translation2d desiredSpeeds =
        this.getLinearVelocityTranslationFromJoysticks(
            controller.getLeftY(), controller.getLeftX());
    this.state = "DRIVE_ALIGN_ANGLE_JOY";
    this.driveFieldOrientedLockedJoystickAngle(
        desiredSpeeds,
        controller.notUsingJoystick() ? 0 : controller.getCOS_Joystick(),
        controller.notUsingJoystick() ? 0 : controller.getSIN_Joystick());
  }

  public void driveRotating(boolean rotateRight) {
    ChassisSpeeds desiredSpeeds =
        this.getLinearVelocityChassisSpeedsFromJoysticks(
            controller.getLeftY(), controller.getLeftX(), rotateRight ? -1.5 : 1.5);
    this.state = "DRIVE_ALIGN_ANGLE_ROTATING_RIGHT?:" + Boolean.toString(rotateRight);
    this.driveFieldOrientedAllianceBased(desiredSpeeds);
  }

  public void driveHeadingBackwards() {
    ChassisSpeeds desiredSpeeds =
        this.getLinearVelocityChassisSpeedsFromJoysticks(
            controller.getLeftY(), controller.getLeftX());
    Rotation2d targetRotation =
        controller.notUsingJoystick()
            ? Rotation2d.kZero
            : new Rotation2d(controller.getCOS_Joystick(), controller.getCOS_Joystick());
    this.state = "DRIVE_HEADING_FORWARD";
    this.driveFieldOrientedLockedAngle(desiredSpeeds, targetRotation, true);
  }

  public void periodic() {
    super.periodic();
    LimelightHelpers.SetRobotOrientation(
        VisionConstants.backLeftLimelightName,
        OdometryEnabledSwerveSubsystem.robotOrientation,
        0,
        0,
        0,
        0,
        0);
    LimelightHelpers.SetRobotOrientation(
        VisionConstants.backRightLimelightName,
        OdometryEnabledSwerveSubsystem.robotOrientation,
        0,
        0,
        0,
        0,
        0);
  }

  public boolean isAtTargetPositionWithHeading() {
    return stableAtTargetPose.isStableInCondition(
        () ->
            isAtTargetPose(
                this.goToPoseTranslationDeadband,
                this.goToPoseTranslationDeadband,
                this.goToPoseHeadingDeadband));
  }

  public boolean isAtTargetPositionWithoutHeading() {
    return stableAtTargetPose.isStableInCondition(
        () -> isAtTargetPose(this.goToPoseTranslationDeadband, this.goToPoseTranslationDeadband));
  }

  public void stopSwerve() {
    this.driveRobotOriented(new ChassisSpeeds());
    this.state = "STOP_SWERVE";
  }

  public void driveAlignAngleJoystickSuperSlow() {
    ChassisSpeeds desiredSpeeds =
        this.getLinearVelocityChassisSpeedsFromJoysticks(
                controller.getLeftY(), controller.getLeftX())
            .times(0.15);
    this.state = "DRIVE_ALIGN_ANGLE_JOY_SUPER_SLOW";
    this.driveFieldOrientedLockedJoystickAngle(
        desiredSpeeds, controller.getCOS_Joystick(), controller.getSIN_Joystick());
  }

  public void driveAimingAtPoseTest() {
    Pose2d target = new Pose2d(8.272, 4.132, new Rotation2d());
    this.state = "DRIVE_AIMING_POSITION_TEST";
    Translation2d desiredSpeeds =
        this.getLinearVelocityTranslationFromJoysticks(
            controller.getLeftY(), controller.getLeftX());
    this.driveAimingAtPosition(desiredSpeeds, target.getTranslation());
  }

  public Command runSysIdQuasistaticForward() {
    return super.sysIdQuasistatic(Direction.kForward);
  }

  public Command runSysIdQuasistaticReversed() {
    return super.sysIdQuasistatic(Direction.kReverse);
  }

  public Command runSysIdDynamicForward() {
    return super.sysIdDynamic(Direction.kForward);
  }

  public Command runSysIdDynamicReversed() {
    return super.sysIdDynamic(Direction.kReverse);
  }

  public Command runWheelRadiusCharacterization() {
    return super.wheelRadiusCharacterization();
  }

  public void goToClimbPoseRight() {
    if (DriverStation.getAlliance() != null) {
      super.driveToPose(
          DriverStation.getAlliance().get() == Alliance.Blue
              ? FieldConstants.FieldStructurePoses.BLUE_RIGHT_TOWER
              : FieldConstants.FieldStructurePoses.RED_RIGHT_TOWER);
    } else {
      super.driveToPose(FieldConstants.FieldStructurePoses.BLUE_RIGHT_TOWER);
    }
  }

  public void goToClimbPoseLeft() {
    if (DriverStation.getAlliance() != null) {
      super.driveToPose(
          DriverStation.getAlliance().get() == Alliance.Blue
              ? FieldConstants.FieldStructurePoses.BLUE_LEFT_TOWER
              : FieldConstants.FieldStructurePoses.RED_LEFT_TOWER);
    } else {
      super.driveToPose(FieldConstants.FieldStructurePoses.BLUE_LEFT_TOWER);
    }
  }
}
