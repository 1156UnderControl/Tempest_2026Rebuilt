package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.java_is_undercontrol.motors.MotorIOInputsAutoLogged;
import frc.java_is_undercontrol.motors.SparkFlexMotor;
import frc.java_is_undercontrol.motors.SparkMAXMotor;
import frc.java_is_undercontrol.motors.TalonFXMotor;
import frc.robot.constants.FieldConstants.FieldStructurePoses;
import frc.robot.constants.shooter.FlyWheelsConstants;
import frc.robot.constants.shooter.HoodConstants;
import frc.robot.constants.shooter.TurretConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private SwerveSubsystem swerve;
  private Pose2d robotPose;
  private boolean isHubMode;

  private final TalonFXMotor flyWheelsMotorLead;
  private final TalonFXMotor flyWheelsMotorFollower;
  private final SparkFlexMotor turretMotor;
  private final SparkMAXMotor hoodMotor;

  private final DCMotorSim flyWheelsMotorLeadSim;
  private final DCMotorSim flyWheelsMotorFollowerSim;
  private final DCMotorSim turretMotorSim;
  private final DCMotorSim hoodMotorSim;

  private final ShooterIOInputsAutoLogged shooterInputs;
  private final MotorIOInputsAutoLogged flyWheelsLeadInputs;
  private final MotorIOInputsAutoLogged flyWheelsFollowerInputs;
  private final MotorIOInputsAutoLogged turretInputs;
  private final MotorIOInputsAutoLogged hoodInputs;

  public ShooterSubsystem(SwerveSubsystem swerve) {
    this.swerve = swerve;
    this.robotPose = new Pose2d();

    this.isHubMode = true;

    this.shooterInputs = new ShooterIOInputsAutoLogged();
    this.flyWheelsLeadInputs = new MotorIOInputsAutoLogged();
    this.flyWheelsFollowerInputs = new MotorIOInputsAutoLogged();
    this.turretInputs = new MotorIOInputsAutoLogged();
    this.hoodInputs = new MotorIOInputsAutoLogged();

    this.flyWheelsMotorLead =
        new TalonFXMotor(
            FlyWheelsConstants.FLY_WHEELS_LEAD_MOTOR_ID, CANBus.roboRIO(), "FlyWheelsLead");
    this.flyWheelsMotorFollower =
        new TalonFXMotor(
            FlyWheelsConstants.FLY_WHEELS_FOLLOWER_MOTOR_ID, CANBus.roboRIO(), "FlyWheelsFollower");
    this.turretMotor = new SparkFlexMotor(TurretConstants.TURRET_MOTOR_ID, "Turret");
    this.hoodMotor = new SparkMAXMotor(HoodConstants.HOOD_MOTOR_ID, "Hood");

    this.flyWheelsMotorFollower.setFollower(FlyWheelsConstants.FLY_WHEELS_LEAD_MOTOR_ID, isHubMode);

    this.turretMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                TurretConstants.TURRET_MOTOR,
                TurretConstants.MOMENT_OF_INERTIA,
                TurretConstants.GEARBOX),
            TurretConstants.TURRET_MOTOR);
    this.flyWheelsMotorLeadSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                FlyWheelsConstants.FLYWHEELS_MOTOR,
                FlyWheelsConstants.MOMENT_OF_INERTIA,
                FlyWheelsConstants.GEARBOX),
            FlyWheelsConstants.FLYWHEELS_MOTOR);
    this.flyWheelsMotorFollowerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                FlyWheelsConstants.FLYWHEELS_MOTOR,
                FlyWheelsConstants.MOMENT_OF_INERTIA,
                FlyWheelsConstants.GEARBOX),
            FlyWheelsConstants.FLYWHEELS_MOTOR);
    this.hoodMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                HoodConstants.HOOD_MOTOR, HoodConstants.MOMENT_OF_INERTIA, HoodConstants.GEARBOX),
            HoodConstants.HOOD_MOTOR);
  }

  @Override
  public void periodic() {
    this.robotPose = this.swerve.getOdometryPose();
    this.simulationPeriodic();
    this.updateInputs();
  }

  @Override
  public void simulationPeriodic() {
    turretMotorSim.setInput(
        (turretMotor.getDutyCycleSetpoint()) * RobotController.getBatteryVoltage());
    flyWheelsMotorLeadSim.setInput(
        (flyWheelsMotorLead.getDutyCycleSetpoint()) * RobotController.getBatteryVoltage());
    flyWheelsMotorFollowerSim.setInput(
        (flyWheelsMotorFollower.getDutyCycleSetpoint()) * RobotController.getBatteryVoltage());
    hoodMotorSim.setInput((hoodMotor.getDutyCycleSetpoint()) * RobotController.getBatteryVoltage());
    turretMotorSim.update(0.02);
    flyWheelsMotorLeadSim.update(0.02);
    flyWheelsMotorFollowerSim.update(0.02);
    hoodMotorSim.update(0.02);
  }

  private void updateInputs() {
    shooterInputs.distanceToTargetAngle = getDistanceToTargetPose();
    shooterInputs.shooterTargetFieldAngle = getTurretTargetAngle();
    shooterInputs.isAtTargetHeading = isShooterPrepared();

    flyWheelsMotorLead.updateInputs(flyWheelsLeadInputs);
    flyWheelsMotorFollower.updateInputs(flyWheelsFollowerInputs);
    turretMotor.updateInputs(turretInputs);
    hoodMotor.updateInputs(hoodInputs);

    Logger.processInputs("Subsystems/Shooter/", shooterInputs);
    Logger.processInputs("Motor/Shooter/FlyWheelLead", flyWheelsLeadInputs);
    Logger.processInputs("Motor/Shooter/FlyWheelFollower", flyWheelsFollowerInputs);
    Logger.processInputs("Motor/Shooter/Turret", turretInputs);
    Logger.processInputs("Motor/Shooter/Hood", hoodInputs);
  }


  public void setTargetMode(boolean isHubMode) {
    this.isHubMode = isHubMode;
  }


  public void prepareToShoot() {
    double targetAngle = this.getTurretTargetAngle();
    this.turretMotor.setPositionReference(targetAngle);
    this.getHoodTargetAngle(this.getDistanceToTargetPose());
  }

  public boolean isShooterPrepared() {
    return isTurretPrepared() && isFlyWheelsPrepared() && isHoodPrepared();
  }


  public void shootFuel() {
    flyWheelsMotorLead.set(getFlyWheelsTargetVelocity(getDistanceToTargetPose()));
  }


  public SubsystemBase getCurrentSubsystem() {
    return this;
  }

  private boolean isTurretPrepared() {
    return this.turretMotor.getPosition()
            <= TurretConstants.ANGLE_ERROR_ALLOWED + getTurretTargetAngle()
        && this.turretMotor.getPosition()
            >= -TurretConstants.ANGLE_ERROR_ALLOWED + getTurretTargetAngle();
  }

  // Tem que mexer depois que quisermos colocar velocidade
  private boolean isFlyWheelsPrepared() {
    return this.flyWheelsMotorLead.getAppliedOutput()
            <= FlyWheelsConstants.VELOCITY_ERROR_ALLOWED
                + getFlyWheelsTargetVelocity(getDistanceToTargetPose())
        && this.flyWheelsMotorLead.getAppliedOutput()
            >= -FlyWheelsConstants.VELOCITY_ERROR_ALLOWED
                + getFlyWheelsTargetVelocity(getDistanceToTargetPose());
  }

  private boolean isHoodPrepared() {
    return this.hoodMotor.getPosition()
            <= HoodConstants.ANGLE_ERROR_ALLOWED + getHoodTargetAngle(getDistanceToTargetPose())
        && this.hoodMotor.getPosition()
            >= -HoodConstants.ANGLE_ERROR_ALLOWED + getHoodTargetAngle(getDistanceToTargetPose());
  }

  private double getTurretTargetAngle() {
    double alphaAngle = this.getFieldAngleToTarget();
    double robotHeading = this.swerve.getOdometryPose().getRotation().getDegrees();
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
    if (targetAngle > TurretConstants.TURRET_MAX_ANGLE) {
      return targetAngle - 400;
    } else if (targetAngle < TurretConstants.TURRET_MIN_ANGLE) {
      return targetAngle + 400;
    } else {
      return targetAngle;
    }
  }

  private double getXSideFromTriangle() {
    return this.getTargetAlliancePose().getX()
        - this.getShooterFieldPoseFromRobotCenter(TurretConstants.TURRET_OFFSET_FROM_ROBOT_CENTER)
            .getX();
  }

  private double getYSideFromTriangle() {
    return this.getTargetAlliancePose().getY()
        - this.getShooterFieldPoseFromRobotCenter(TurretConstants.TURRET_OFFSET_FROM_ROBOT_CENTER)
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
    Translation2d rotatedOffset = turretOffset.rotateBy(this.robotPose.getRotation());
    return new Pose2d(
        this.robotPose.getTranslation().plus(rotatedOffset), this.robotPose.getRotation());
  }
}
