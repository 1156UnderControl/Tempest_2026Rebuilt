package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants.FieldStructurePoses;
import frc.robot.constants.shooter.FlyWheelsConstants;
import frc.robot.constants.shooter.HoodConstants;
import frc.robot.constants.shooter.TurretConstants;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXSWrapper;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private final Supplier<Pose2d> robotPose;
  private boolean isHubMode;

  private final SmartMotorControllerConfig turretConfig;
  private final SparkFlex turretMotor;
  private final SmartMotorController smartTurretMotor;
  private final PivotConfig turretMechanismConfig;
  private final Pivot turretMechanism;

  private final SmartMotorControllerConfig hoodConfig;
  private final SparkMax hoodMotor;
  private final SmartMotorController smartHoodMotor;
  private final ArmConfig hoodMechanismConfig;
  private final Arm hoodMechanism;

  private final SmartMotorControllerConfig flyWheelLeadConfig;
  private final TalonFXS flyWheelLeadMotor;
  private final SmartMotorController smartFlyWheelLeadMotor;
  private final FlyWheelConfig flyWheelLeadMechanismConfig;
  private final FlyWheel flyWheelLeadMechanism;

  private final SmartMotorControllerConfig flyWheelFollowerConfig;
  private final TalonFXS flyWheelFollowerMotor;
  private final SmartMotorController smartFlyWheelFollowerMotor;
  private final FlyWheelConfig flyWheelFollowerMechanismConfig;
  private final FlyWheel flyWheelFollowerMechanism;

  private final ShooterInputsAutoLogged shooterInputs;

  public ShooterSubsystem(Supplier<Pose2d> robotPose) {
    this.robotPose = robotPose;
    this.isHubMode = true;

    this.turretConfig = new SmartMotorControllerConfig(this)
      .withIdleMode(TurretConstants.MOTOR_MODE)
      .withGearing(TurretConstants.MOTOR_GEARING)
      .withMotorInverted(TurretConstants.Position.MOTOR_INVERTED)
      .withTelemetry(TurretConstants.MOTOR_NAME, TurretConstants.TELEMETRY)
      .withSoftLimit(TurretConstants.Position.TURRET_MIN_ANGLE, TurretConstants.Position.TURRET_MAX_ANGLE)
      .withStatorCurrentLimit(TurretConstants.STATOR_CURRENT)
      .withClosedLoopRampRate(TurretConstants.CLOSED_RAMP_RATE)
      .withClosedLoopController(TurretConstants.ClosedLoopController.P,TurretConstants.ClosedLoopController.I,TurretConstants.ClosedLoopController.D, TurretConstants.Velocity.MAX_VELOCITY, TurretConstants.Velocity.ACCELERATION)
      .withFeedforward(new SimpleMotorFeedforward(TurretConstants.FeedForward.KS, TurretConstants.FeedForward.KV, TurretConstants.FeedForward.KA))
      .withControlMode(TurretConstants.CONTROL_MODE);

    this.turretMotor = new SparkFlex(TurretConstants.TURRET_MOTOR_ID, TurretConstants.MOTOR_TYPE);

    this.smartTurretMotor = new SparkWrapper(this.turretMotor, TurretConstants.TURRET_MOTOR, this.turretConfig);

    this.turretMechanismConfig = new PivotConfig(smartTurretMotor)
      .withStartingPosition(TurretConstants.Position.STARTING_POSITION)
      .withHardLimit(TurretConstants.Position.TURRET_MIN_ANGLE, TurretConstants.Position.TURRET_MAX_ANGLE)
      .withMOI(TurretConstants.TURRET_SIZE, TurretConstants.TURRET_MASS);

    this.turretMechanism = new Pivot(turretMechanismConfig);



    this.hoodConfig = new SmartMotorControllerConfig(this)
      .withIdleMode(HoodConstants.MOTOR_MODE)
      .withGearing(HoodConstants.MOTOR_GEARING)
      .withMotorInverted(HoodConstants.Position.MOTOR_INVERTED)
      .withTelemetry(HoodConstants.MOTOR_NAME, HoodConstants.TELEMETRY)
      .withSoftLimit(HoodConstants.Position.HOOD_MIN_ANGLE, HoodConstants.Position.HOOD_MAX_ANGLE)
      .withStatorCurrentLimit(HoodConstants.STATOR_CURRENT)
      .withClosedLoopRampRate(HoodConstants.CLOSED_RAMP_RATE)
      .withClosedLoopController(HoodConstants.ClosedLoopController.P,HoodConstants.ClosedLoopController.I,HoodConstants.ClosedLoopController.D, HoodConstants.Velocity.MAX_VELOCITY, HoodConstants.Velocity.ACCELERATION)
      .withFeedforward(new SimpleMotorFeedforward(HoodConstants.FeedForward.KS, HoodConstants.FeedForward.KV, HoodConstants.FeedForward.KA))
      .withControlMode(HoodConstants.CONTROL_MODE);

    this.hoodMotor = new SparkMax(HoodConstants.HOOD_MOTOR_ID, HoodConstants.MOTOR_TYPE);

    this.smartHoodMotor = new SparkWrapper(this.hoodMotor, HoodConstants.HOOD_MOTOR, this.hoodConfig);

    this.hoodMechanismConfig = new ArmConfig(smartHoodMotor)
      .withStartingPosition(HoodConstants.Position.STARTING_POSITION)
      .withHardLimit(HoodConstants.Position.HOOD_MIN_ANGLE, HoodConstants.Position.HOOD_MAX_ANGLE);

    this.hoodMechanism = new Arm(hoodMechanismConfig);



    this.flyWheelLeadConfig = new SmartMotorControllerConfig(this)
      .withIdleMode(FlyWheelsConstants.MOTOR_MODE)
      .withGearing(FlyWheelsConstants.MOTOR_GEARING)
      .withMotorInverted(FlyWheelsConstants.Position.MOTOR_INVERTED_LEAD)
      .withTelemetry(FlyWheelsConstants.MOTOR_LEAD_NAME, FlyWheelsConstants.TELEMETRY)
      .withStatorCurrentLimit(FlyWheelsConstants.STATOR_CURRENT)
      .withClosedLoopRampRate(FlyWheelsConstants.CLOSED_RAMP_RATE)
      .withClosedLoopController(FlyWheelsConstants.ClosedLoopController.P,FlyWheelsConstants.ClosedLoopController.I,FlyWheelsConstants.ClosedLoopController.D, FlyWheelsConstants.Velocity.MAX_VELOCITY, FlyWheelsConstants.Velocity.ACCELERATION)
      .withFeedforward(new SimpleMotorFeedforward(FlyWheelsConstants.FeedForward.KS, FlyWheelsConstants.FeedForward.KV, FlyWheelsConstants.FeedForward.KA))
      .withControlMode(FlyWheelsConstants.CONTROL_MODE);

    this.flyWheelLeadMotor = new TalonFXS(FlyWheelsConstants.FLYWHEEL_MOTOR_LEAD_ID);

    this.smartFlyWheelLeadMotor = new TalonFXSWrapper(this.flyWheelLeadMotor, FlyWheelsConstants.FLYWHEEL_MOTOR, this.flyWheelLeadConfig);

    this.flyWheelLeadMechanismConfig = new FlyWheelConfig(smartFlyWheelLeadMotor)
      .withMOI(FlyWheelsConstants.FLYWHEEL_SIZE, FlyWheelsConstants.FLYWHEEL_MASS);

    this.flyWheelLeadMechanism = new FlyWheel(flyWheelLeadMechanismConfig);



    this.flyWheelFollowerConfig = new SmartMotorControllerConfig(this)
      .withIdleMode(FlyWheelsConstants.MOTOR_MODE)
      .withGearing(FlyWheelsConstants.MOTOR_GEARING)
      .withMotorInverted(FlyWheelsConstants.Position.MOTOR_INVERTED_FOLLOWER)
      .withTelemetry(FlyWheelsConstants.MOTOR_FOLLOWER_NAME, FlyWheelsConstants.TELEMETRY)
      .withStatorCurrentLimit(FlyWheelsConstants.STATOR_CURRENT)
      .withClosedLoopRampRate(FlyWheelsConstants.CLOSED_RAMP_RATE)
      .withClosedLoopController(FlyWheelsConstants.ClosedLoopController.P,FlyWheelsConstants.ClosedLoopController.I,FlyWheelsConstants.ClosedLoopController.D, FlyWheelsConstants.Velocity.MAX_VELOCITY, FlyWheelsConstants.Velocity.ACCELERATION)
      .withFeedforward(new SimpleMotorFeedforward(FlyWheelsConstants.FeedForward.KS, FlyWheelsConstants.FeedForward.KV, FlyWheelsConstants.FeedForward.KA))
      .withControlMode(FlyWheelsConstants.CONTROL_MODE);

    this.flyWheelFollowerMotor = new TalonFXS(FlyWheelsConstants.FLYWHEEL_MOTOR_LEAD_ID);

    this.smartFlyWheelFollowerMotor = new TalonFXSWrapper(this.flyWheelFollowerMotor, FlyWheelsConstants.FLYWHEEL_MOTOR, this.flyWheelFollowerConfig);

    this.flyWheelFollowerMechanismConfig = new FlyWheelConfig(smartFlyWheelFollowerMotor)
      .withMOI(FlyWheelsConstants.FLYWHEEL_SIZE, FlyWheelsConstants.FLYWHEEL_MASS);

    this.flyWheelFollowerMechanism = new FlyWheel(flyWheelFollowerMechanismConfig);

    this.shooterInputs = new ShooterInputsAutoLogged();
  }

  @Override
  public void periodic() {
    this.smartFlyWheelFollowerMotor.setDutyCycle(smartFlyWheelLeadMotor.getDutyCycle());
    this.updateInputs();
    this.turretMechanism.updateTelemetry();
    this.hoodMechanism.updateTelemetry();
    this.flyWheelLeadMechanism.updateTelemetry();
    this.flyWheelFollowerMechanism.updateTelemetry();
  }

  private void updateInputs() {
    this.shooterInputs.distanceToTargetAngle = getDistanceToTargetPose();
    this.shooterInputs.shooterTargetFieldAngle = getTurretTargetAngle();
    this.shooterInputs.isAtTargetHeading = isShooterPrepared();

    Logger.processInputs("Subsystems/Shooter/", shooterInputs);
  }

  @Override
  public void simulationPeriodic(){
    this.turretMechanism.simIterate();
    this.hoodMechanism.simIterate();
    this.flyWheelLeadMechanism.simIterate();
    this.flyWheelFollowerMechanism.simIterate();
  }

  public void setTargetMode(boolean isHubMode) {
    this.isHubMode = isHubMode;
  }

  public void prepareToShoot() {
    double targetAngle = this.getTurretTargetAngle();
    this.smartTurretMotor.setPosition(Angle.ofBaseUnits(targetAngle, Degrees));
    this.getHoodTargetAngle(this.getDistanceToTargetPose());
  }

  public boolean isShooterPrepared() {
    return isTurretPrepared() && isFlyWheelsPrepared() && isHoodPrepared();
  }

  public void shootFuel() {
    smartFlyWheelLeadMotor.setDutyCycle(getFlyWheelsTargetVelocity(getDistanceToTargetPose()));
  }

  public SubsystemBase getCurrentSubsystem() {
    return this;
  }

  private boolean isTurretPrepared() {
    return this.smartTurretMotor.getMechanismPosition().in(Degrees)
            <= HoodConstants.Position.ANGLE_ERROR_ALLOWED + getTurretTargetAngle()
        && this.smartTurretMotor.getMechanismPosition().in(Degrees)
            >= -HoodConstants.Position.ANGLE_ERROR_ALLOWED + getTurretTargetAngle();
  }

  // Tem que mexer depois que quisermos colocar velocidade
  private boolean isFlyWheelsPrepared() {
    return this.smartFlyWheelLeadMotor.getDutyCycle()
            <= FlyWheelsConstants.Velocity.VELOCITY_ERROR_ALLOWED
                + getFlyWheelsTargetVelocity(getDistanceToTargetPose())
        && this.smartFlyWheelLeadMotor.getDutyCycle()
            >= -FlyWheelsConstants.Velocity.VELOCITY_ERROR_ALLOWED
                + getFlyWheelsTargetVelocity(getDistanceToTargetPose());
  }

  private boolean isHoodPrepared() {
    return this.smartHoodMotor.getMechanismPosition().in(Degrees)
            <= HoodConstants.Position.ANGLE_ERROR_ALLOWED + getHoodTargetAngle(getDistanceToTargetPose())
        && this.smartHoodMotor.getMechanismPosition().in(Degrees)
            >= -HoodConstants.Position.ANGLE_ERROR_ALLOWED + getHoodTargetAngle(getDistanceToTargetPose());
  }

  private double getTurretTargetAngle() {
    double alphaAngle = this.getFieldAngleToTarget();
    double robotHeading = this.robotPose.get().getRotation().getDegrees();
    double turretTargetAngle = alphaAngle - robotHeading;

    return getReacheableTurretAngle(turretTargetAngle);
  }

  // Tem que mexer quando mexermos na interpolação, vai ser de velocidade
  private double getFlyWheelsTargetVelocity(double distanceToTargetPose) {
    return 0.5;
  }

  // Tem que mexer quando mexermos na interpolação
  private double getHoodTargetAngle(double distanceToTargetPose) {
    return 45;
  }

  private double getDistanceToTargetPose() {
    return Math.hypot(this.getXSideFromTriangle(), this.getYSideFromTriangle());
  }

  private double getReacheableTurretAngle(double targetAngle) {
    if (targetAngle > TurretConstants.Position.TURRET_MAX_ANGLE.in(Degrees)) {
      return targetAngle - 400;
    } else if (targetAngle < TurretConstants.Position.TURRET_MIN_ANGLE.in(Degrees)) {
      return targetAngle + 400;
    } else {
      return targetAngle;
    }
  }

  private double getXSideFromTriangle() {
    return this.getTargetAlliancePose().getX()
        - this.getShooterFieldPoseFromRobotCenter(TurretConstants.Position.TURRET_OFFSET_FROM_ROBOT_CENTER)
            .getX();
  }

  private double getYSideFromTriangle() {
    return this.getTargetAlliancePose().getY()
        - this.getShooterFieldPoseFromRobotCenter(TurretConstants.Position.TURRET_OFFSET_FROM_ROBOT_CENTER)
            .getY();
  }

  private Pose2d getTargetAlliancePose() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue); // valor padrão

    if (this.isHubMode) {
      return alliance == Alliance.Blue
          ? FieldStructurePoses.BLUE_HUB_POSE
          : FieldStructurePoses.RED_HUB_POSE;
    } else {
      return alliance == Alliance.Blue
          ? FieldStructurePoses.BLUE_HUB_POSE
          : FieldStructurePoses.RED_HUB_POSE;
    }
  }

  private double getFieldAngleToTarget() {
    double alphaAngle = Math.atan2(this.getYSideFromTriangle(), this.getXSideFromTriangle());
    return Math.toDegrees(alphaAngle);
  }

  private Pose2d getShooterFieldPoseFromRobotCenter(Translation2d turretOffset) {
    Translation2d rotatedOffset = turretOffset.rotateBy(this.robotPose.get().getRotation());
    return new Pose2d(
        this.robotPose.get().getTranslation().plus(rotatedOffset),
        this.robotPose.get().getRotation());
  }
}
