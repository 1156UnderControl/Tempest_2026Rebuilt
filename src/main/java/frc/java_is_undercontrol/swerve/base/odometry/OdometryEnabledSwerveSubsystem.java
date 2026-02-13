package frc.java_is_undercontrol.swerve.base.odometry;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.java_is_undercontrol.swerve.SwerveDevices;
import frc.java_is_undercontrol.swerve.base.BaseSwerveSubsystem;
import frc.java_is_undercontrol.swerve.configs.OdometryEnabledSwerveConfig;
import frc.java_is_undercontrol.vision.odometry.PoseEstimation;
import frc.java_is_undercontrol.vision.odometry.PoseEstimator;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public abstract class OdometryEnabledSwerveSubsystem extends BaseSwerveSubsystem {

  public static double robotOrientation;

  public static double robotAngularVelocity;

  private PoseEstimator autonomousPoseEstimator;

  private PoseEstimator teleopPoseEstimator;

  private PoseEstimator defaultAutonomousPoseEstimator;

  private PoseEstimator defaultTeleopPoseEstimator;

  private PIDController moveToPoseXAxisPid;

  private PIDController moveToPoseYAxisPid;

  @AutoLogOutput private Pose2d targetPose;

  private Pose2d targetAimPose;

  // Create the constraints to use while pathfinding. The constraints defined in
  // the path will only be used for the path.
  PathConstraints constraints;

  public OdometryEnabledSwerveSubsystem(OdometryEnabledSwerveConfig config, SwerveDevices devices) {
    super(devices, config);
    this.moveToPoseXAxisPid = config.moveToPosePIDConfig.getPidTranslation();
    this.moveToPoseYAxisPid = config.moveToPosePIDConfig.getPidTranslation();
    this.constraints = config.pathPlannerConfig.pathFinderConstraints;
    this.autonomousPoseEstimator = config.autonomousPoseEstimator;
    this.teleopPoseEstimator = config.teleoperatedPoseEstimator;
    this.defaultAutonomousPoseEstimator = config.teleoperatedPoseEstimator;
    this.defaultAutonomousPoseEstimator = config.autonomousPoseEstimator;
    this.defaultTeleopPoseEstimator = config.teleoperatedPoseEstimator;
    this.targetPose = new Pose2d();
    this.targetAimPose = new Pose2d();
  }

  private void updateOdometry() {
    if (DriverStation.isAutonomousEnabled() || DriverStation.isAutonomous()) {
      this.updateOdometryWithPoseEstimator(this.autonomousPoseEstimator);
    } else {
      this.updateOdometryWithPoseEstimator(this.teleopPoseEstimator);
    }
  }

  protected void overrideTeleOpPoseEstimator(PoseEstimator poseEstimator) {
    if (poseEstimator == null) {
      this.teleopPoseEstimator = this.defaultTeleopPoseEstimator;
      return;
    }
    this.teleopPoseEstimator = poseEstimator;
  }

  protected void overrideAutonomousPoseEstimator(PoseEstimator poseEstimator) {
    if (poseEstimator == null) {
      this.autonomousPoseEstimator = this.defaultAutonomousPoseEstimator;
      return;
    }
    this.autonomousPoseEstimator = poseEstimator;
  }

  private void updateOdometryWithPoseEstimator(PoseEstimator poseEstimator) {
    Pose2d referencePose = getPose();
    Optional<PoseEstimation> possibleEstimatedPose = poseEstimator.getEstimatedPose(referencePose);
    if (possibleEstimatedPose.isPresent()) {
      PoseEstimation estimatedPose = possibleEstimatedPose.get();
      Pose2d poseVision = estimatedPose.estimatedPose.toPose2d();
      Logger.recordOutput("Odometry/PoseVision", poseVision);
      this.addVisionMeasurement(
          poseVision, estimatedPose.timestampSeconds, estimatedPose.visionStdDev);
    }
  }

  protected double getDistanceToPosition(Translation2d targetPosition) {
    return this.getPose().getTranslation().getDistance(targetPosition);
  }

  protected void driveAimingAtPosition(ChassisSpeeds targetSpeeds, Translation2d targetPosition) {
    this.driveAimingAtPosition(targetSpeeds, targetPosition, Rotation2d.fromDegrees(0));
  }

  protected void driveAimingAtPosition(Translation2d targetSpeeds, Translation2d targetPosition) {
    this.driveAimingAtPosition(
        new ChassisSpeeds(targetSpeeds.getX(), targetSpeeds.getY(), 0),
        targetPosition,
        Rotation2d.fromDegrees(0));
  }

  protected void driveAimingAtPosition(
      ChassisSpeeds targetSpeeds, Translation2d targetPosition, Rotation2d offsetAngle) {
    this.driveAimingAtPosition(0, targetSpeeds, targetPosition, offsetAngle);
  }

  protected void driveAimingAtPosition(
      double dt, ChassisSpeeds targetSpeeds, Translation2d targetPosition, Rotation2d offsetAngle) {
    Rotation2d targetAngle = this.getHeadingAimingAtPosition(dt, targetPosition, offsetAngle);
    this.driveFieldOrientedLockedAngle(targetSpeeds, targetAngle, true);
  }

  protected void driveAimingAtPosition(
      double dt, ChassisSpeeds targetSpeeds, Translation2d targetPosition) {
    this.driveAimingAtPosition(dt, targetSpeeds, targetPosition, Rotation2d.fromDegrees(0));
  }

  protected void driveAimingAtPosition(
      double dt, Translation2d targetSpeeds, Translation2d targetPosition) {
    this.driveAimingAtPosition(
        dt,
        new ChassisSpeeds(targetSpeeds.getX(), targetSpeeds.getY(), 0),
        targetPosition,
        Rotation2d.fromDegrees(0));
  }

  protected Rotation2d getHeadingAimingAtPosition(
      double dt, Translation2d targetPosition, Rotation2d offsetAngle) {
    this.targetAimPose = new Pose2d(targetPosition, new Rotation2d());
    Translation2d positionDifference =
        this.getEarlyPoseMoving(dt).getTranslation().minus(targetPosition);
    Rotation2d targetAngle = positionDifference.getAngle().plus(offsetAngle);
    return targetAngle;
  }

  protected void driveToPose(Pose2d targetPose) {
    this.driveToPose(targetPose, Double.POSITIVE_INFINITY);
  }

  protected void driveToPoseAimingAtPosition(
      double dt, Pose2d targetPose, Translation2d targetTranslationToAim, double maxSpeed) {
    this.driveToPoseAimingAtPosition(dt, targetPose, targetTranslationToAim, maxSpeed, 0.0);
  }

  protected void driveToPoseAimingAtPosition(
      double dt,
      Pose2d targetPose,
      Translation2d targetTranslationToAim,
      double maxSpeed,
      double offset) {
    Rotation2d targetAngle =
        this.getHeadingAimingAtPosition(dt, targetTranslationToAim, Rotation2d.fromDegrees(offset));
    Pose2d targetPoseWithLockedAngle =
        new Pose2d(targetPose.getX(), targetPose.getY(), targetAngle);
    this.driveToPose(targetPoseWithLockedAngle, maxSpeed);
  }

  protected void driveToPose(Pose2d targetPose, double maxSpeed) {
    Pose2d currentPose = this.getPose();
    double targetXVelocity =
        this.moveToPoseXAxisPid.calculate(currentPose.getX(), (targetPose.getX()));
    double targetYVelocity =
        this.moveToPoseYAxisPid.calculate(currentPose.getY(), (targetPose.getY()));
    double targetTranslationVelocity =
        new Translation2d(targetXVelocity, targetYVelocity).getNorm();
    if (targetTranslationVelocity > maxSpeed) {
      double conversionFactor = maxSpeed / targetTranslationVelocity;
      targetXVelocity = targetXVelocity * conversionFactor;
      targetYVelocity = targetYVelocity * conversionFactor;
    }
    ChassisSpeeds desiredSpeeds = new ChassisSpeeds(targetXVelocity, targetYVelocity, 0);
    this.driveFieldOrientedLockedAngle(desiredSpeeds, targetPose.getRotation(), false);
    this.targetPose = targetPose;
  }

  protected boolean isAtTargetPose(
      double xAxisToleranceMeters, double yAxisToleranceMeters, double angleToleranceDegrees) {
    double xAxisDistance = Math.abs(this.getPose().getX() - this.targetPose.getX());
    double yAxisDistance = Math.abs(this.getPose().getY() - this.targetPose.getY());
    boolean isAtXAxis = xAxisDistance <= xAxisToleranceMeters;
    boolean isAtYAxis = yAxisDistance <= yAxisToleranceMeters;
    boolean isAtAngle = this.isAtTargetHeading(angleToleranceDegrees);
    boolean isAtTargetPose = isAtXAxis && isAtYAxis && isAtAngle;
    return isAtTargetPose;
  }

  protected boolean isAtTargetPose(double xAxisToleranceMeters, double yAxisToleranceMeters) {
    double xAxisDistance = Math.abs(this.getPose().getX() - this.targetPose.getX());
    double yAxisDistance = Math.abs(this.getPose().getY() - this.targetPose.getY());
    boolean isAtXAxis = xAxisDistance <= xAxisToleranceMeters;
    boolean isAtYAxis = yAxisDistance <= yAxisToleranceMeters;
    boolean isAtTargetPose = isAtXAxis && isAtYAxis;
    return isAtTargetPose;
  }

  protected boolean isAtTargetPose(
      Pose2d poseTarget, double xAxisToleranceMeters, double yAxisToleranceMeters) {
    double xAxisDistance = Math.abs(this.getPose().getX() - poseTarget.getX());
    double yAxisDistance = Math.abs(this.getPose().getY() - poseTarget.getY());
    boolean isAtXAxis = xAxisDistance <= xAxisToleranceMeters;
    boolean isAtYAxis = yAxisDistance <= yAxisToleranceMeters;
    boolean isAtTargetPose = isAtXAxis && isAtYAxis;
    return isAtTargetPose;
  }

  @Override
  protected void driveFieldOriented(ChassisSpeeds targetSpeeds) {
    super.driveFieldOriented(targetSpeeds);
  }

  protected void driveFieldOriented(Translation2d targetTranslation, double omega) {
    ChassisSpeeds targetSpeeds =
        new ChassisSpeeds(targetTranslation.getX(), targetTranslation.getY(), omega);
    super.driveFieldOriented(targetSpeeds);
  }

  @Override
  protected void driveRobotOriented(ChassisSpeeds targetSpeeds) {
    super.driveRobotOriented(targetSpeeds);
  }

  protected Command driveToPoseWithPathfinding(Pose2d targetPose) {
    Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
    return pathfindingCommand;
  }

  protected Command driveToPathWithPathfinding(PathPlannerPath path) {
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
    return pathfindingCommand;
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public Command wheelRadiusCharacterization() {
    SlewRateLimiter limiter = new SlewRateLimiter(0.25);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();
    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(5);
                  this.driveRobotOriented(new ChassisSpeeds(0.0, 0.0, speed));
                  System.out.println(this.driveBaseRadius);
                },
                this)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1),
            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = super.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = this.getHeading();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = this.getHeading();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = super.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  protected class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  @Override
  public void periodic() {
    this.updateLimelightRequiredValues();
    this.updateOdometry();
    super.periodic();
  }

  private void updateLimelightRequiredValues() {
    OdometryEnabledSwerveSubsystem.robotAngularVelocity =
        this.getRobotVelocity().omegaRadiansPerSecond;
    OdometryEnabledSwerveSubsystem.robotOrientation = this.getHeading().getDegrees();
  }
}
