package frc.robot.subsystems.swerve;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.java_is_undercontrol.control.ProfiledPIDConfig;
import frc.java_is_undercontrol.swerve.configs.MoveToPosePIDConfig;
import frc.java_is_undercontrol.swerve.configs.OdometryEnabledSwerveConfig;
import frc.java_is_undercontrol.swerve.configs.SwerveDeadbandsConfig;
import frc.java_is_undercontrol.swerve.configs.SwervePathPlannerConfig;
import frc.java_is_undercontrol.swerve.base.odometry.OdometryEnabledSwerveSubsystem;
import frc.java_is_undercontrol.swerve.SwerveDevices;
import frc.java_is_undercontrol.util.StabilizeChecker;
import frc.java_is_undercontrol.vision.deprecated.Cameras.LimelightHelpers;
import frc.java_is_undercontrol.vision.odometry.LimelightPoseEstimator;
import frc.java_is_undercontrol.vision.odometry.MultiCameraPoseEstimator;
import frc.java_is_undercontrol.vision.odometry.PoseEstimation;
import frc.java_is_undercontrol.vision.odometry.PoseEstimator;
import frc.robot.constants.JoystickConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.joysticks.DriverController;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
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
            new SwerveDeadbandsConfig(JoystickConstants.DEADBAND, 0.5),
            pathPlannerConfig,
            configureMulticameraPoseEstimation(),
            configureMulticameraPoseEstimation(),
            new ProfiledPIDConfig(7, 0, 0, 80, 50),
            new MoveToPosePIDConfig(
                SwerveConstants.MOVE_TO_POSE_TRANSLATION_PID,
                SwerveConstants.MOVE_TO_POSE_Y_CONSTRAINTS)),
        swerveDevices);
    this.resetOdometry(new Pose2d());
  }

  private static PoseEstimator configureMulticameraPoseEstimation() {
    List<PoseEstimator> listOfEstimators = new ArrayList<PoseEstimator>();
    PoseEstimator limelightLeft = new LimelightPoseEstimator("limelight-left", false, false, 2);
    PoseEstimator limelightRight = new LimelightPoseEstimator("limelight-right", false, false, 2);
    listOfEstimators.add(limelightLeft);
    listOfEstimators.add(limelightRight);
    PoseEstimator estimatorMultiCamera =
        new MultiCameraPoseEstimator(listOfEstimators, "Teleop Multi Pose Estimator");
    return estimatorMultiCamera;
  }

  @Override
  public void resetOdometryLimelight(Translation2d defaultPosition) {
    PoseEstimator limelightLeft = new LimelightPoseEstimator("limelight-left", false, false, 2);
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
        this.getLinearVelocityFromJoysticks(controller.getXtranslation(), controller.getYtranslation());
    this.state = "DRIVE_ALIGN_ANGLE_JOY";
    this.driveFieldOrientedLockedJoystickAngle(
        desiredSpeeds, controller.getCOS_Joystick(), controller.getSIN_Joystick());
  }

  public void driveRotating(boolean rotateRight) {
    ChassisSpeeds desiredSpeeds =
        this.inputsToChassisSpeeds(
            controller.getXtranslation(), controller.getYtranslation(), rotateRight ? -1.5 : 1.5);
    this.state = "DRIVE_ALIGN_ANGLE_ROTATING_RIGHT?:" + Boolean.toString(rotateRight);
    this.driveFieldOriented(desiredSpeeds);
  }

  @Override
  public void periodic() {
    super.periodic();
    LimelightHelpers.SetRobotOrientation(
        "limelight-left",
        OdometryEnabledSwerveSubsystem.robotOrientation,
        OdometryEnabledSwerveSubsystem.robotAngularVelocity,
        0,
        0,
        0,
        0);
    LimelightHelpers.SetRobotOrientation(
        "limelight-right",
        OdometryEnabledSwerveSubsystem.robotOrientation,
        OdometryEnabledSwerveSubsystem.robotAngularVelocity,
        0,
        0,
        0,
        0);
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
        this.inputsToChassisSpeeds(controller.getXtranslation(), controller.getYtranslation()).times(0.12);
    this.state = "DRIVE_ALIGN_ANGLE_JOY_SUPER_SLOW";
    this.driveFieldOrientedLockedJoystickAngle(
        desiredSpeeds, controller.getCOS_Joystick(), controller.getSIN_Joystick());
  }

  @Override
  public void driveAimingAtPoseTest() {
    Pose2d target = new Pose2d(8.272, 4.132, new Rotation2d());
    this.state = "DRIVE_AIMING_POSITION_TEST";
    Translation2d desiredSpeeds =
        this.getLinearVelocityFromJoysticks(controller.getXtranslation(), controller.getYtranslation());
    this.driveAimingAtPosition(desiredSpeeds, target.getTranslation());
  }

  @Override
  public Command runSysIdQuasistaticForward(){
    System.out.println("comecei a rodar");
    return super.sysIdQuasistatic(Direction.kForward);
  }

  @Override
  public Command runSysIdQuasistaticReversed(){
    return super.sysIdQuasistatic(Direction.kReverse);
  }

  @Override
  public Command runSysIdDynamicForward(){
    return super.sysIdDynamic(Direction.kForward);
  }

  @Override
  public Command runSysIdDynamicReversed(){
    return super.sysIdDynamic(Direction.kReverse);
  }
}
