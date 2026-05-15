// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.java_is_undercontrol.swerve.SwerveDevices;
import frc.java_is_undercontrol.swerve.io.gyro.GyroIO;
import frc.java_is_undercontrol.swerve.io.gyro.GyroIOPigeon2;
import frc.java_is_undercontrol.swerve.io.module.ModuleIO;
import frc.java_is_undercontrol.swerve.io.module.ModuleIOSim;
import frc.java_is_undercontrol.swerve.io.module.ModuleIOTalonFX;
import frc.robot.commands.swerve.PassThroughTower;
import frc.robot.commands.swerve.PassThroughTrenchLeftAlliance;
import frc.robot.generated.TunerConstants;
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.IDriverController;
import frc.robot.joysticks.IOperatorController;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.hopper.kicker.KickerSubsystem;
import frc.robot.subsystems.hopper.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.intake.extensor.ExtensorSubsystem;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlyWheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final States states;

  private final SwerveSubsystem drive;
  private final ClimberSubsystem climber;
  private final TurretSubsystem turret;
  private final HoodSubsystem hood;
  private final FlyWheelSubsystem flyWheel;
  private final SpindexerSubsystem spindexer;
  private final KickerSubsystem kicker;
  private final RollerSubsystem roller;
  private final ExtensorSubsystem extensor;
  private final IDriverController driverController;
  private final IOperatorController operatorController;

  private final NamedCommandsRegistery namedCommandsRegistry;

  private final SwerveDevices simulatedSwerveDevices =
      new SwerveDevices(
          new GyroIO() {},
          new ModuleIOSim(TunerConstants.FrontLeft),
          new ModuleIOSim(TunerConstants.FrontRight),
          new ModuleIOSim(TunerConstants.BackLeft),
          new ModuleIOSim(TunerConstants.BackRight));

  private final SwerveDevices talonFxSwervedevices =
      new SwerveDevices(
          new GyroIOPigeon2(),
          new ModuleIOTalonFX(TunerConstants.FrontLeft),
          new ModuleIOTalonFX(TunerConstants.FrontRight),
          new ModuleIOTalonFX(TunerConstants.BackLeft),
          new ModuleIOTalonFX(TunerConstants.BackRight));

  private final SwerveDevices replayedDevices =
      new SwerveDevices(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {});

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    this.driverController = DriverController.getInstance();
    this.operatorController = OperatorController.getInstance();

    switch (Constants.getMode()) {
      case REAL:
        drive = new SwerveSubsystem(talonFxSwervedevices);
        break;

      case SIM:
        drive = new SwerveSubsystem(simulatedSwerveDevices);
        break;

      default:
        drive = new SwerveSubsystem(replayedDevices);
        break;
    }

    this.climber = ClimberSubsystem.getInstance();
    this.kicker = KickerSubsystem.getInstance();
    this.turret = TurretSubsystem.getInstance(this.climber.getAbsoluteEncoder());
    this.hood = HoodSubsystem.getInstance();
    this.flyWheel = FlyWheelSubsystem.getInstance();
    this.spindexer = SpindexerSubsystem.getInstance();
    this.roller = RollerSubsystem.getInstance();
    this.extensor = ExtensorSubsystem.getInstance();

    this.namedCommandsRegistry =
        new NamedCommandsRegistery(drive, roller, extensor, kicker, spindexer, climber);
    this.namedCommandsRegistry.registerAllAutoCommands();

    this.states =
        new States(
            this.drive,
            this.turret,
            this.hood,
            this.flyWheel,
            this.spindexer,
            this.kicker,
            this.roller,
            this.extensor,
            this.climber);

    configureButtonBindings();
    configAutoTriggers();
    this.autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  }

  private void configureButtonBindings() {
    this.states.setDefaultStates();
    this.driverController
        .turboActivate()
        .onTrue(this.drive.boostRobotJoystickControlVelocity().ignoringDisable(true))
        .onFalse(this.drive.limitRobotJoystickControlVelocity().ignoringDisable(true));

    this.driverController
        .rightBumper()
        .and(
            () ->
                !RobotStates.getInstance().isOnNonOdometryMode()
                    // && !RobotStates.getInstance().isOnNotShootZone()
                    && this.hood.isAtTargetAngle()
                    && this.turret.isAtTargetAngle()
                    && this.flyWheel.isAtTargetVelocity())
        .whileTrue(
            Commands.parallel(
                this.states.shootState(),
                Commands.runOnce(() -> RobotStates.getInstance().setShootingMode(true))))
        .onFalse(Commands.runOnce(() -> RobotStates.getInstance().setShootingMode(false)));


    this.driverController
        .leftBumper()
        .whileTrue(Commands.parallel(this.states.shootState(), this.flyWheel.runLowVelocity(), this.hood.trackNonOdometry(), this.turret.trackNonOdometry()));


    this.driverController
        .a()
        .onTrue(
            Commands.parallel(
                this.states.collectState(),
                Commands.runOnce(() -> RobotStates.getInstance().setExtendedMode(true))));

    this.driverController.x().whileTrue(this.states.shakeState());

    this.driverController
        .y()
        .onTrue(
            Commands.parallel(
                this.states.retractState(),
                Commands.runOnce(() -> RobotStates.getInstance().setExtendedMode(false))));

    new Trigger(() -> RobotStates.getInstance().isExtendedMode())
        .and(this.driverController.x().negate())
        .whileTrue(this.states.collectState());

    this.driverController.upArrow().onTrue(this.states.climbState());
    this.driverController.downArrow().onTrue(this.states.defaultClimberState());



    // this.operatorController
    //     .l2()
    //     .onTrue(Commands.runOnce(() -> SwerveSubsystem.runVisionHdr()).ignoringDisable(true));
    // this.operatorController
    //     .r3()
    //     .onTrue(Commands.runOnce(() ->
    // SwerveSubsystem.stopVisionHdrDark()).ignoringDisable(true));
    // this.operatorController
    //     .l3()
    //     .onTrue(
    //         Commands.runOnce(() -> SwerveSubsystem.stopVisionHdrBright()).ignoringDisable(true));



    new Trigger(
            () ->
                RobotStates.getInstance().getShouldConfigRollerAuto()
                    && !RobotStates.getInstance().getShouldConfigRollerTeleop())
        .onTrue(Commands.runOnce(() -> this.roller.configureAuto()))
        .onFalse(Commands.runOnce(() -> this.roller.configureTeleop()));

    new Trigger(() -> RobotStates.getInstance().isShootingMode() && !DriverStation.isAutonomous())
        .onTrue(Commands.runOnce(() -> this.roller.configureShooting()))
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (RobotStates.getInstance().getShouldConfigRollerAuto()
                      && !RobotStates.getInstance().getShouldConfigRollerTeleop()) {
                    this.roller.configureAuto();
                  } else {
                    this.roller.configureTeleop();
                  }
                }));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void configAutoTriggers() {
    new Trigger(
            () ->
                !RobotStates.getInstance().isOnNonOdometryMode()
                    // && !RobotStates.getInstance().isOnNotShootZone()
                    && this.hood.isAtTargetAngle()
                    && this.turret.isAtTargetAngle()
                    && this.flyWheel.isAtTargetVelocity()
                    && DriverStation.isAutonomousEnabled()
                    && RobotStates.getInstance().getShouldShootMode())
        .whileTrue(
            Commands.parallel(
                this.states.shootState(),
                Commands.runOnce(() -> RobotStates.getInstance().setShootingMode(true))))
        .onFalse(Commands.runOnce(() -> RobotStates.getInstance().setShootingMode(false)));

    new Trigger(
            () ->
                DriverStation.isAutonomousEnabled()
                    && RobotStates.getInstance().isOnAllianceZone()
                    && !RobotStates.getInstance().isOnNotShootZone())
        .whileTrue(this.flyWheel.shootHub());
    new Trigger(
            () ->
                DriverStation.isAutonomousEnabled()
                    && !RobotStates.getInstance().isOnAllianceZone()
                    && !RobotStates.getInstance().isOnNotShootZone())
        .whileTrue(this.flyWheel.shootPass());

    new Trigger(() -> DriverStation.isTeleopEnabled())
        .onTrue(Commands.runOnce(() -> this.drive.configureDriveMode(true)));
  }
}
