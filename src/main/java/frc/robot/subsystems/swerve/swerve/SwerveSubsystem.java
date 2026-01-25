package frc.robot.subsystems.swerve.swerve;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
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
import frc.java_is_undercontrol.vision.deprecated.Cameras.LimelightHelpers;
import frc.java_is_undercontrol.vision.odometry.LimelightPoseEstimator;
import frc.java_is_undercontrol.vision.odometry.MultiCameraPoseEstimator;
import frc.java_is_undercontrol.vision.odometry.PoseEstimation;
import frc.java_is_undercontrol.vision.odometry.PoseEstimator;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.JoystickConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.joysticks.DriverController;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class SwerveSubsystem extends OdometryEnabledSwerveSubsystem implements ISwerve {

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

  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;

  private static final SwervePathPlannerConfig pathPlannerConfig =
      new SwervePathPlannerConfig(
          new PIDConstants(5, 0, 0),
          new PIDConstants(5, 0, 0),
          new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));

  public SwerveSubsystem(SwerveDevices swerveDevices) {
    super(
        new OdometryEnabledSwerveConfig(
            0.75,
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
    PoseEstimator limelightCenter = new LimelightPoseEstimator("limelight-center", false, false, 2);
    listOfEstimators.add(limelightCenter);
    PoseEstimator estimatorMultiCamera =
        new MultiCameraPoseEstimator(listOfEstimators, "Teleop Multi Pose Estimator");
    return estimatorMultiCamera;
  }

  @Override
  public void resetOdometryLimelight(Translation2d defaultPosition) {
    PoseEstimator limelightLeft = new LimelightPoseEstimator("limelight-center", false, false, 2);
    Optional<PoseEstimation> limelightPoseEstimation =
        limelightLeft.getEstimatedPose(this.getPose());
    if (limelightPoseEstimation.isEmpty()) {
      resetTranslation(defaultPosition);
      this.positionUpdated = true;
    } else {
      resetTranslation(
          limelightPoseEstimation.get().estimatedPose.getTranslation().toTranslation2d());
      this.positionUpdated = true;
    }
  }

  public Supplier<Pose2d> getOdometryPose() {
    return this.robotPose;
  }

  public Supplier<ChassisSpeeds> getCurrentSpeeds() {
    return this.currentSpeeds;
  }

  @Override
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
        this.getLinearVelocityFromJoysticksTranslation(
            MathUtil.applyDeadband(controller.getLeftY(), JoystickConstants.DEADBAND_TRANSLATION),
            MathUtil.applyDeadband(controller.getLeftX(), JoystickConstants.DEADBAND_TRANSLATION));
    this.state = "DRIVE_ALIGN_ANGLE_JOY";
    this.driveFieldOrientedLockedJoystickAngle(
        desiredSpeeds,
        controller.notUsingJoystick() ? 0 : controller.getCOS_Joystick(),
        controller.notUsingJoystick() ? 0 : controller.getSIN_Joystick());
  }

  public void driveRotating(boolean rotateRight) {
    ChassisSpeeds desiredSpeeds =
        this.inputsToChassisSpeeds(
            controller.getLeftY(), controller.getLeftX(), rotateRight ? -1.5 : 1.5);
    this.state = "DRIVE_ALIGN_ANGLE_ROTATING_RIGHT?:" + Boolean.toString(rotateRight);
    this.driveFieldOriented(desiredSpeeds);
  }

  @Override
  public void periodic() {
    super.periodic();
    LimelightHelpers.SetRobotOrientation(
        "limelight-center", OdometryEnabledSwerveSubsystem.robotOrientation, 0, 0, 0, 0, 0);
  }

  @Override
  public boolean isAtTargetPositionWithHeading() {
    return stableAtTargetPose.isStableInCondition(
        () ->
            isAtTargetPose(
                this.goToPoseTranslationDeadband,
                this.goToPoseTranslationDeadband,
                this.goToPoseHeadingDeadband));
  }

  @Override
  public boolean isAtTargetPositionWithoutHeading() {
    return stableAtTargetPose.isStableInCondition(
        () -> isAtTargetPose(this.goToPoseTranslationDeadband, this.goToPoseTranslationDeadband));
  }

  @Override
  public void stopSwerve() {
    this.driveRobotOriented(new ChassisSpeeds());
    this.state = "STOP_SWERVE";
  }

  @Override
  public void driveAlignAngleJoystickSuperSlow() {

    ChassisSpeeds desiredSpeeds =
        this.getLinearVelocityFromJoysticksChassisSpeeds(
                controller.getLeftY(), controller.getLeftX())
            .times(0.15);
    this.state = "DRIVE_ALIGN_ANGLE_JOY_SUPER_SLOW";
    this.driveFieldOrientedLockedJoystickAngle(
        desiredSpeeds, controller.getCOS_Joystick(), controller.getSIN_Joystick());
  }

  @Override
  public void driveAimingAtPoseTest() {
    Pose2d target = new Pose2d(8.272, 4.132, new Rotation2d());
    this.state = "DRIVE_AIMING_POSITION_TEST";
    Translation2d desiredSpeeds =
        this.getLinearVelocityFromJoysticksTranslation(
            controller.getLeftY(), controller.getLeftX());
    this.driveAimingAtPosition(desiredSpeeds, target.getTranslation());
  }

  @Override
  public Command runSysIdQuasistaticForward() {
    return super.sysIdQuasistatic(Direction.kForward);
  }

  @Override
  public Command runSysIdQuasistaticReversed() {
    return super.sysIdQuasistatic(Direction.kReverse);
  }

  @Override
  public Command runSysIdDynamicForward() {
    return super.sysIdDynamic(Direction.kForward);
  }

  @Override
  public Command runSysIdDynamicReversed() {
    return super.sysIdDynamic(Direction.kReverse);
  }

  @Override
  public Command runWheelRadiusCharacterization() {
    return super.wheelRadiusCharacterization();
  }

  @Override
  public void goToClimbPoseRight() {
    if (DriverStation.getAlliance() != null) {
      super.driveToPose(
          DriverStation.getAlliance().get() == Alliance.Blue
              ? FieldConstants.FieldStructurePoses.BLUE_RIGHT_TOWER
              : FieldConstants.FieldStructurePoses.RED_RIGHT_TOWER);
    } else {
      super.driveToPose(FieldConstants.FieldStructurePoses.RED_RIGHT_TOWER);
    }
  }

  @Override
  public void goToClimbPoseLeft() {
    if (DriverStation.getAlliance() != null) {
      super.driveToPose(
          DriverStation.getAlliance().get() == Alliance.Blue
              ? FieldConstants.FieldStructurePoses.BLUE_LEFT_TOWER
              : FieldConstants.FieldStructurePoses.RED_LEFT_TOWER);
    } else {
      super.driveToPose(FieldConstants.FieldStructurePoses.RED_LEFT_TOWER);
    }
  }
}
