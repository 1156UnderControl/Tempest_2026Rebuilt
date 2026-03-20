package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

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
import frc.java_is_undercontrol.vision.odometry.PhotonPoseEstimatorStrategies;
import frc.java_is_undercontrol.vision.odometry.PhotonVisionPoseEstimator;
import frc.java_is_undercontrol.vision.odometry.PoseEstimator;
import frc.robot.RobotStates;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.JoystickConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.joysticks.DriverController;
import java.util.ArrayList;
import java.util.List;
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

  @AutoLogOutput protected String state = "NULL";

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

  private boolean isUsingTowerEstimation;

  private static MultiCameraPoseEstimator poseEstimatorGeneral;
  private static LimelightPoseEstimator poseEstimatorBackLeft;
  private static LimelightPoseEstimator poseEstimatorBackRight;
  private static PhotonVisionPoseEstimator poseEstimatorFrontRight;
  private static PhotonVisionPoseEstimator poseEstimatorFrontLeft;

  public SwerveSubsystem(SwerveDevices swerveDevices) {
    super(
        new OdometryEnabledSwerveConfig(
            0.75,
            0.60,
            new SwerveDeadbandsConfig(
                JoystickConstants.DEADBAND_TRANSLATION, JoystickConstants.DEADBAND_ROTATION),
            pathPlannerConfig,
            configureMulticameraPoseEstimation(),
            configureMulticameraPoseEstimation(),
            new ProfiledPIDConfig(5.0, 0, 0, 0, 10000),
            new MoveToPosePIDConfig(
                SwerveConstants.MOVE_TO_POSE_TRANSLATION_PID,
                SwerveConstants.MOVE_TO_POSE_Y_CONSTRAINTS)),
        swerveDevices);
    this.resetOdometry(new Pose2d());
    this.robotPose = () -> super.getPose();
    this.currentSpeeds = () -> super.getRobotVelocity();
    this.isUsingTowerEstimation = false;
  }

  private static PoseEstimator configureMulticameraPoseEstimation() {
    List<PoseEstimator> listOfEstimators = new ArrayList<PoseEstimator>();
    SwerveSubsystem.poseEstimatorBackLeft =
        new LimelightPoseEstimator(VisionConstants.backLeftLimelightName, false, false, 2);

    SwerveSubsystem.poseEstimatorBackRight =
        new LimelightPoseEstimator(VisionConstants.backRightLimelightName, false, false, 2);

    PhotonCamera cameraFrontRight = new PhotonCamera(VisionConstants.frontRightArducamName);
    SwerveSubsystem.poseEstimatorFrontRight =
        new PhotonVisionPoseEstimator(
            cameraFrontRight, VisionConstants.robotToCamArducamFrontRight);

    PhotonCamera cameraFrontLeft = new PhotonCamera(VisionConstants.frontLeftArducamName);
    SwerveSubsystem.poseEstimatorFrontLeft =
        new PhotonVisionPoseEstimator(
            cameraFrontLeft,
            VisionConstants.robotToCamArducamFrontLeft,
            false,
            1.5,
            PhotonPoseEstimatorStrategies.MULTITAG_COPROCESSOR);

    listOfEstimators.add(SwerveSubsystem.poseEstimatorBackLeft);
    listOfEstimators.add(SwerveSubsystem.poseEstimatorBackRight);
    listOfEstimators.add(SwerveSubsystem.poseEstimatorFrontRight);
    listOfEstimators.add(SwerveSubsystem.poseEstimatorFrontLeft);

    if (SwerveSubsystem.poseEstimatorGeneral == null) {
      SwerveSubsystem.poseEstimatorGeneral =
          new MultiCameraPoseEstimator(listOfEstimators, "Teleop Multi Pose Estimator");
    }

    return SwerveSubsystem.poseEstimatorGeneral;
  }

  public Supplier<Pose2d> getOdometryPose() {
    return this.robotPose;
  }

  public Supplier<ChassisSpeeds> getCurrentSpeeds() {
    return this.currentSpeeds;
  }

  public void driveAlignAngleJoystick() {

    Translation2d desiredSpeeds =
        this.getLinearVelocityTranslationFromJoysticks(
            controller.getLeftY(), controller.getLeftX());
    this.state = "DRIVE_ALIGN_ANGLE_JOY";
    this.driveFieldOrientedLockedJoystickAngle(
        desiredSpeeds,
        controller.notUsingJoystick() ? 0 : controller.getCOS_Joystick(),
        controller.notUsingJoystick() ? 0 : controller.getSIN_Joystick());
  }

  public void driveAlignAngleJoystickYfixedOnTrench() {
    Rotation2d nearestAngleToLock;
    if (getPose().getRotation().getDegrees() > 90 || getPose().getRotation().getDegrees() < -90) {
      nearestAngleToLock = new Rotation2d(Degrees.of(90));
    } else {
      nearestAngleToLock = new Rotation2d(Degrees.of(-90));
    }

    boolean isRedAlliance =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Translation2d desiredSpeeds =
        isRedAlliance
            ? this.getLinearVelocityTranslationFromJoysticks(-controller.getLeftY(), 0)
            : this.getLinearVelocityTranslationFromJoysticks(controller.getLeftY(), 0);

    this.state = "DRIVE_ALIGN_TRENCH";

    boolean isOnDepotSide =
        isRedAlliance
            ? RobotStates.getInstance().isOnRedDepotSide()
            : RobotStates.getInstance().isOnBlueDepotSide();
    if (isRedAlliance) {
      if (getPose().getRotation().getDegrees() > 90 || getPose().getRotation().getDegrees() < -90) {
        nearestAngleToLock = new Rotation2d(Degrees.of(90));
      } else {
        nearestAngleToLock = new Rotation2d(Degrees.of(-90));
      }
    } else {
      if (getPose().getRotation().getDegrees() > 90 || getPose().getRotation().getDegrees() < -90) {
        nearestAngleToLock = new Rotation2d(Degrees.of(-90));
      } else {
        nearestAngleToLock = new Rotation2d(Degrees.of(90));
      }
    }

    double trenchY;
    if (isRedAlliance) {
      trenchY =
          isOnDepotSide
              ? FieldConstants.FieldStructurePoses.SAFE_ALLIANCE_TRENCH_POSE_LEFT_RED.getY()
              : FieldConstants.FieldStructurePoses.SAFE_ALLIANCE_TRENCH_POSE_RIGHT_RED.getY();
    } else {
      trenchY =
          isOnDepotSide
              ? FieldConstants.FieldStructurePoses.SAFE_ALLIANCE_TRENCH_POSE_LEFT_BLUE.getY()
              : FieldConstants.FieldStructurePoses.SAFE_ALLIANCE_TRENCH_POSE_RIGHT_BLUE.getY();
    }

    double speed =
        controller.turboActivate().getAsBoolean()
            ? TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
            : 2.5;
    boolean usingJoystick = !controller.notUsingJoystick();
    double cosValue = usingJoystick ? controller.getCOS_Joystick() : nearestAngleToLock.getCos();
    double sinValue = usingJoystick ? controller.getSIN_Joystick() : nearestAngleToLock.getSin();

    super.driveToSpecificYinField(trenchY, speed, desiredSpeeds, cosValue, sinValue);
  }

  public void driveAlignAngleJoystickYfixedOnTower() {
    Rotation2d nearestAngleToLock;

    boolean isRedAlliance =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Translation2d desiredSpeeds =
        isRedAlliance
            ? this.getLinearVelocityTranslationFromJoysticks(0, -controller.getLeftX())
            : this.getLinearVelocityTranslationFromJoysticks(0, controller.getLeftX());

    this.state = "DRIVE_ALIGN_TOWER";

    if (isRedAlliance) {
      if (getPose().getRotation().getDegrees() < 0) {
        nearestAngleToLock = new Rotation2d(Degrees.of(0));
      } else {
        nearestAngleToLock = new Rotation2d(Degrees.of(180));
      }
    } else {
      if (getPose().getRotation().getDegrees() < 0) {
        nearestAngleToLock = new Rotation2d(Degrees.of(180));
      } else {
        nearestAngleToLock = new Rotation2d(Degrees.of(0));
      }
    }

    double towerX;
    if (isRedAlliance) {
      towerX = FieldConstants.FieldStructurePoses.TOWER_POSE_RED.getX();
    } else {
      towerX = FieldConstants.FieldStructurePoses.TOWER_POSE_BLUE.getX();
    }

    double speed =
        controller.turboActivate().getAsBoolean()
            ? TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
            : 2.5;
    boolean usingJoystick = !controller.notUsingJoystick();
    double cosValue = usingJoystick ? controller.getCOS_Joystick() : nearestAngleToLock.getCos();
    double sinValue = usingJoystick ? controller.getSIN_Joystick() : nearestAngleToLock.getSin();

    super.driveToSpecificXinField(towerX, speed, desiredSpeeds, cosValue, sinValue);
  }

  public Rotation2d getNearestBumpAngle() {
    Rotation2d nearestAngleToLock;
    if (getPose().getRotation().getDegrees() > 90 || getPose().getRotation().getDegrees() < -90) {
      if (getPose().getRotation().getDegrees() < -90) {
        nearestAngleToLock = new Rotation2d(Degrees.of(-135));
      } else {
        nearestAngleToLock = new Rotation2d(Degrees.of(135));
      }
    } else {
      if (getPose().getRotation().getDegrees() < 0) {
        nearestAngleToLock = new Rotation2d(Degrees.of(-45));
      } else {
        nearestAngleToLock = new Rotation2d(Degrees.of(45));
      }
    }
    return nearestAngleToLock;
  }

  public void driveAlignBump(Rotation2d lockedAngle) {
    ChassisSpeeds desiredSpeeds =
        this.getLinearVelocityChassisSpeedsFromJoysticks(
            controller.getLeftY(), controller.getLeftX());
    this.state = "DRIVE_ALIGN_ANGLE_BUMP";
    this.driveFieldOrientedLockedAngle(desiredSpeeds, lockedAngle, true);
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
    // this.verifyClimbPoseEstimation();
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

  public void driveAlignAngleJoystickShootWhileMovingVelocity() {
    ChassisSpeeds desiredSpeeds =
        this.getLinearVelocityChassisSpeedsFromJoysticks(
                controller.getLeftY(), controller.getLeftX())
            .times(0.60);
    this.state = "DRIVE_SUPER_SLOW";
    this.driveFieldOrientedLockedJoystickAngle(
        desiredSpeeds, controller.getCOS_Joystick(), controller.getSIN_Joystick());
  }

  public void driveAlignAngleJoystickSuperSlow() {
    ChassisSpeeds desiredSpeeds =
        this.getLinearVelocityChassisSpeedsFromJoysticks(
                controller.getLeftY(), controller.getLeftX())
            .times(0.30);
    this.state = "DRIVE_SUPER_SLOW";
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

  public boolean isAtSafezoneForGoToClimb() {
    return (getPose().getY() > 5.1 || getPose().getY() < 3)
        && (getPose().getX() < 2 || getPose().getX() > 14.5);
  }

  public boolean isAtRightClimb() {
    if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
      return getPose().getY() > 4.3;
    }
    return getPose().getY() < 3.7;
  }

  public void goToClimbPoseRight() {
    this.state = "GO_TO_CLIMB_RIGHT";
    Pose2d targetPose;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      targetPose = FieldConstants.FieldStructurePoses.RED_RIGHT_TOWER;
    } else {
      targetPose = FieldConstants.FieldStructurePoses.BLUE_RIGHT_TOWER;
    }
    if (DriverStation.isTeleop()) {
      super.driveToPose(targetPose, 1.5);
    } else {
      super.driveToPose(targetPose);
    }
  }

  public void goToClimbPoseLeft() {
    this.state = "GO_TO_CLIMB_LEFT";
    Pose2d targetPose;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      targetPose = FieldConstants.FieldStructurePoses.RED_LEFT_TOWER;
    } else {
      targetPose = FieldConstants.FieldStructurePoses.BLUE_LEFT_TOWER;
    }
    if (DriverStation.isTeleop()) {
      super.driveToPose(targetPose, 1.5);
    } else {
      super.driveToPose(targetPose);
    }
  }

  public void setAutoPoseEstimatorToDefault() {
    overrideAutonomousPoseEstimator(null);
  }

  private void verifyClimbPoseEstimation() {
    if (isUsingTowerEstimation) {
      poseEstimatorBackLeft.onlySeeApriltagsIDs(FieldConstants.AprilTags.towerTags);
      poseEstimatorBackRight.onlySeeApriltagsIDs(FieldConstants.AprilTags.towerTags);
      poseEstimatorFrontRight.ignoreApriltags(FieldConstants.AprilTags.withoutTowerTags);
    } else {
      poseEstimatorBackLeft.considerAllTags();
      poseEstimatorBackRight.considerAllTags();
      poseEstimatorFrontRight.considerAllTags();
    }
  }

  public void setPoseEstimatorToClimbOnAuto(boolean climbingOnRight) {
    if (climbingOnRight) {
      this.overrideAutonomousPoseEstimator(poseEstimatorBackRight);
    } else {
      this.overrideAutonomousPoseEstimator(poseEstimatorBackLeft);
    }
  }
}
