package frc.java_is_undercontrol.swerve.base;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.java_is_undercontrol.swerve.Drive;
import frc.java_is_undercontrol.swerve.SwerveDevices;
import frc.java_is_undercontrol.swerve.configs.BaseSwerveConfig;
import frc.java_is_undercontrol.swerve.configs.SwervePathPlannerConfig;
import frc.java_is_undercontrol.util.CustomMath;
import frc.java_is_undercontrol.util.Util;
import frc.robot.generated.TunerConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class BaseSwerveSubsystem extends SubsystemBase {

  public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  protected double MaxAngularRate;

  protected final double driveBaseRadius =
      Math.hypot(
          TunerConstants.FrontLeft.LocationX + TunerConstants.FrontRight.LocationX / 2,
          TunerConstants.FrontLeft.LocationY + TunerConstants.BackLeft.LocationY / 2);

  private final SwervePathPlannerConfig pathPlannerConfig;

  private Matrix<N3, N1> actualVisionStdDev = VecBuilder.fill(0.7, 0.7, Double.POSITIVE_INFINITY);

  @AutoLogOutput private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

  @AutoLogOutput private double targetHeadingDegrees = Double.NaN;

  @AutoLogOutput private double targetHeadingRadians = Double.NaN;

  @AutoLogOutput private double lastDesiredJoystickAngleRadians = Double.NaN;

  @AutoLogOutput private boolean robotBoostedOnJoystickControl = false;

  private final double FF_START_DELAY = 2.0; // Secs
  private final double FF_RAMP_RATE = 0.1; // Volts/Sec

  ProfiledPIDController angleController;

  double translationDeadband;
  double joystickAimDeadband;
  double velocityReductionFactor;

  private Drive drive;

  protected BaseSwerveSubsystem(SwerveDevices devices, BaseSwerveConfig config) {

    drive = new Drive(devices, this);

    angleController =
        new ProfiledPIDController(
            config.headingPidConfig.kP,
            config.headingPidConfig.kI,
            config.headingPidConfig.kD,
            new TrapezoidProfile.Constraints(
                drive.getMaxAngularSpeedDegreesPerSecond(), config.headingPidConfig.maxAcceleration));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    pathPlannerConfig = config.pathPlannerConfig;
    configureAutoBuilder();
    this.translationDeadband = config.deadbandsConfig.translationDeadband;
    this.joystickAimDeadband = config.deadbandsConfig.joystickAimDeadband;
    this.velocityReductionFactor = config.velocityReductionFactor;
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return drive.sysIdQuasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return drive.sysIdDynamic(direction);
  }

  private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          drive::setPose, // Method to reset odometry (will be called if your auto has a starting
          // pose)
          drive::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          drive::runVelocity,
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following
              // controller for
              // holonomic drive trains
              pathPlannerConfig.translationPid, // Translation PID constants
              pathPlannerConfig.anglePid // Rotation PID constants
              ),
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    drive.setPose(initialHolonomicPose);
  }

  public void resetTranslation(Translation2d translationToReset) {
    Pose2d poseToReset = new Pose2d(translationToReset, getHeading());
    this.resetOdometry(poseToReset);
  }

  public void zeroGyro() {
    Pose2d poseToReset = new Pose2d(getPose().getTranslation(), new Rotation2d());
    this.resetOdometry(poseToReset);
  }

  public void setMotorBrake(boolean brake) {
    if (brake) {
      drive.setBrakeToModules();
    } else {
      drive.setCoastToModules();
    }
  }

  protected Pose2d getPose() {
    return drive.getPose();
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    drive.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    if (visionMeasurementStdDevs.isEqual(actualVisionStdDev, 0)) {
      return;
    }
    actualVisionStdDev = visionMeasurementStdDevs;
  }

  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    this.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  protected Pose2d getEarlyPoseMoving(double dt) {
    Pose2d actualPose = getPose();
    Translation2d earlyTranslation =
        new Translation2d(
            actualPose.getX() + getFieldOrientedRobotVelocity().vxMetersPerSecond * dt,
            actualPose.getY() + getFieldOrientedRobotVelocity().vyMetersPerSecond * dt);
    Pose2d earlyPoseMoving = new Pose2d(earlyTranslation, actualPose.getRotation());
    return earlyPoseMoving;
  }

  protected Pose2d getEarlyPoseMovingWithOmega(double dt) {
    Pose2d actualPose = getPose();
    Translation2d earlyTranslation =
        new Translation2d(
            actualPose.getX() + getFieldOrientedRobotVelocity().vxMetersPerSecond * dt,
            actualPose.getY() + getFieldOrientedRobotVelocity().vyMetersPerSecond * dt);
    Rotation2d earlyRotation =
        new Rotation2d(
            actualPose.getRotation().getRadians()
                + getFieldOrientedRobotVelocity().omegaRadiansPerSecond * dt);
    Pose2d earlyPoseMoving = new Pose2d(earlyTranslation, earlyRotation);
    return earlyPoseMoving;
  }

  protected void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    this.targetSpeeds = chassisSpeeds;
    drive.runVelocity(chassisSpeeds);
  }

  protected ChassisSpeeds getRobotVelocity() {
    return drive.getChassisSpeeds();
  }

  protected ChassisSpeeds getFieldOrientedRobotVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), getHeading());
  }

  protected double getAbsoluteRobotVelocity() {
    return CustomMath.toAbsoluteSpeed(this.getRobotVelocity());
  }

  protected Rotation2d getHeading() {
    return drive.getRotation();
  }

  public void lock() {
    drive.stopWithX();
  }

  @Override
  public void periodic() {
    drive.periodic();
    Logger.recordOutput("TargetHeadingDegrees", this.targetHeadingDegrees);
    Logger.recordOutput("TargetHeadingRadians", this.targetHeadingRadians);
  }

  /**
   * Drives the robot field-oriented while locking its heading to a target angle.
   *
   * <p>The translational components ({@code vx}, {@code vy}) come from the given {@link
   * ChassisSpeeds}. Rotation is controlled automatically using an angle controller to track the
   * desired heading. When on the Red alliance, the target heading is flipped by 180° to maintain
   * consistent field-relative behavior.
   *
   * @param speeds Desired field-relative translational speeds
   * @param targetHeading Target field heading to maintain
   * @param allianceBasedVelocities Whether the heading should account for alliance orientation
   */
  protected void driveFieldOrientedLockedAngle(
      ChassisSpeeds speeds, Rotation2d targetHeading, boolean allianceBasedVelocities) {
    Rotation2d targetRotation = targetHeading;
    this.lastDesiredJoystickAngleRadians = targetRotation.getRadians();
    this.targetHeadingDegrees = targetRotation.getDegrees();
    this.targetHeadingRadians = targetRotation.getRadians();
    double omega =
        angleController.calculate(getHeading().getRadians(), targetRotation.getRadians());
    ChassisSpeeds adjustedSpeeds =
        new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omega);
    if (allianceBasedVelocities) {
      driveFieldOrientedAllianceBased(adjustedSpeeds);
    } else {
      driveFieldOriented(adjustedSpeeds);
    }
  }

  protected Translation2d getLinearVelocityTranslationFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), translationDeadband);
    if (!robotBoostedOnJoystickControl) {
      linearMagnitude = linearMagnitude * velocityReductionFactor;
    }
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    linearMagnitude = linearMagnitude * linearMagnitude;
    Translation2d joystickTranslation =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    return new Translation2d(
        joystickTranslation.getX() * drive.getMaxLinearSpeedMetersPerSec(),
        joystickTranslation.getY() * drive.getMaxLinearSpeedMetersPerSec());
  }

  protected ChassisSpeeds getLinearVelocityChassisSpeedsFromJoysticks(
      double x, double y, double angularRate) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), translationDeadband);
    if (!robotBoostedOnJoystickControl) {
      linearMagnitude = linearMagnitude * velocityReductionFactor;
    }
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    linearMagnitude = linearMagnitude * linearMagnitude;
    Translation2d joystickTranslation =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    return new ChassisSpeeds(
        joystickTranslation.getX() * drive.getMaxLinearSpeedMetersPerSec(),
        joystickTranslation.getY() * drive.getMaxLinearSpeedMetersPerSec(),
        angularRate);
  }

  protected ChassisSpeeds getLinearVelocityChassisSpeedsFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), translationDeadband);
    if (!robotBoostedOnJoystickControl) {
      linearMagnitude = linearMagnitude * velocityReductionFactor;
    }
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    linearMagnitude = linearMagnitude * linearMagnitude;
    Translation2d joystickTranslation =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    return new ChassisSpeeds(
        joystickTranslation.getX() * drive.getMaxLinearSpeedMetersPerSec(),
        joystickTranslation.getY() * drive.getMaxLinearSpeedMetersPerSec(),
        0);
  }

  protected void driveFieldOrientedLockedJoystickAngle(
      ChassisSpeeds speeds, double xHeading, double yHeading, double joystickAimDeadband) {
    if (this.lastDesiredJoystickAngleRadians == Double.NaN) {
      this.lastDesiredJoystickAngleRadians = this.getPose().getRotation().getRadians();
    }
    double angle =
        Util.withinHypotDeadband(xHeading, yHeading, joystickAimDeadband)
            ? lastDesiredJoystickAngleRadians
            : Math.atan2(xHeading, yHeading);
    // It has to be de pre-inverted angle
    this.lastDesiredJoystickAngleRadians = angle;
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    Rotation2d targetRotation =
        isFlipped
            ? Rotation2d.fromRadians(angle).plus(Rotation2d.kPi)
            : Rotation2d.fromRadians(angle);
    Logger.recordOutput("IsFlipped", isFlipped);
    this.targetHeadingDegrees = targetRotation.getDegrees();
    this.targetHeadingRadians = targetRotation.getRadians();
    double omega =
        angleController.calculate(drive.getRotation().getRadians(), targetHeadingRadians);
    ChassisSpeeds adjustedSpeeds =
        new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omega);
    driveFieldOrientedAllianceBased(adjustedSpeeds);
  }

  protected void driveFieldOrientedLockedJoystickAngle(
      ChassisSpeeds speeds, double xHeading, double yHeading) {
    driveFieldOrientedLockedJoystickAngle(speeds, xHeading, yHeading, this.joystickAimDeadband);
  }

  protected void driveFieldOrientedLockedJoystickAngle(
      Translation2d speeds, double xHeading, double yHeading, double joystickAimDeadband) {
    ChassisSpeeds speedsWithTranslation = new ChassisSpeeds(speeds.getX(), speeds.getY(), 0);
    driveFieldOrientedLockedJoystickAngle(
        speedsWithTranslation, xHeading, yHeading, joystickAimDeadband);
  }

  protected void driveFieldOrientedLockedJoystickAngle(
      Translation2d speeds, double xHeading, double yHeading) {
    ChassisSpeeds speedsWithTranslation = new ChassisSpeeds(speeds.getX(), speeds.getY(), 0);
    driveFieldOrientedLockedJoystickAngle(speedsWithTranslation, xHeading, yHeading);
  }

  protected void driveFieldOrientedAllianceBased(ChassisSpeeds speeds) {
    this.targetSpeeds = speeds;
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  protected void driveFieldOriented(ChassisSpeeds speeds) {
    this.targetSpeeds = speeds;
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
  }

  protected void driveRobotOriented(ChassisSpeeds speeds) {
    this.targetSpeeds = speeds;
    this.targetHeadingDegrees = Double.NaN;
    this.targetHeadingRadians = Double.NaN;
    drive.runVelocity(speeds);
  }

  public Command limitRobotJoystickControlVelocity() {
    return Commands.runOnce(() -> this.robotBoostedOnJoystickControl = false);
  }

  public Command boostRobotJoystickControlVelocity() {
    return Commands.runOnce(() -> this.robotBoostedOnJoystickControl = true);
  }

  protected boolean isAtTargetHeading(double toleranceDegrees) {
    if (this.targetHeadingDegrees == Double.NaN) {
      return false;
    }
    Rotation2d targetHeading = Rotation2d.fromDegrees(this.targetHeadingDegrees);
    Rotation2d currentHeading = this.getHeading();
    double headingDifferenceDegrees = Math.abs(targetHeading.minus(currentHeading).getDegrees());
    boolean isAtHeading = headingDifferenceDegrees <= toleranceDegrees;
    return isAtHeading;
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    return drive.getWheelRadiusCharacterizationPositions();
  }

  public Command feedforwardCharacterization() {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterizationDrive(0.0);
                },
                this)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterizationDrive(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                this)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }
}
