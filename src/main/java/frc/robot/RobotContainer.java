// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.java_is_undercontrol.swerve.SwerveDevices;
import frc.java_is_undercontrol.swerve.io.gyro.GyroIO;
import frc.java_is_undercontrol.swerve.io.gyro.GyroIOPigeon2;
import frc.java_is_undercontrol.swerve.io.module.ModuleIO;
import frc.java_is_undercontrol.swerve.io.module.ModuleIOSim;
import frc.java_is_undercontrol.swerve.io.module.ModuleIOTalonFX;
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
    this.turret =
        TurretSubsystem.getInstance(
            this.kicker.getTurreAbsoluteEncoder(), this.climber.getAbsoluteEncoder());
    this.hood = HoodSubsystem.getInstance();
    this.flyWheel = FlyWheelSubsystem.getInstance();
    this.spindexer = SpindexerSubsystem.getInstance();
    this.roller = RollerSubsystem.getInstance();
    this.extensor = ExtensorSubsystem.getInstance();

    this.namedCommandsRegistry = new NamedCommandsRegistery(drive);
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
    this.autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  }

  private void configureButtonBindings() {
    this.states.setDefaultStates();
    this.driverController
        .turboActivate()
        .onTrue(this.drive.boostRobotJoystickControlVelocity())
        .onFalse(this.drive.limitRobotJoystickControlVelocity());

    this.operatorController
        .rightTrigger()
        .and(
            () ->
                RobotStates.getInstance().isOnAllianceZone()
                    && !RobotStates.getInstance().isOnNonOdometryMode()
                    && !RobotStates.getInstance().isOnNotShootZone()
                    && this.hood.isAtTargetAngleShoot())
        .whileTrue(Commands.parallel(this.flyWheel.shootHub(), this.states.shootState()));
    this.operatorController
        .rightTrigger()
        .and(
            () ->
                !RobotStates.getInstance().isOnAllianceZone()
                    && !RobotStates.getInstance().isOnNonOdometryMode()
                    && !RobotStates.getInstance().isOnNotShootZone()
                    && this.hood.isAtTargetAngleShoot())
        .whileTrue(Commands.parallel(this.flyWheel.shootPass(), this.states.shootState()));
    this.operatorController
        .rightTrigger()
        .and(() -> RobotStates.getInstance().isOnNonOdometryMode())
        .whileTrue(this.flyWheel.shootNonOdometry());

    this.operatorController
        .a()
        .onTrue(
            Commands.parallel(
                this.states.collectState(),
                Commands.runOnce(() -> RobotStates.getInstance().setExtendedMode(true))));
    this.operatorController.x().whileTrue(this.states.retractState());
    this.operatorController
        .y()
        .onTrue(
            Commands.parallel(
                this.states.retractState(),
                Commands.runOnce(() -> RobotStates.getInstance().setExtendedMode(false))));

    this.operatorController.rightBumper().whileTrue(this.spindexer.expell());
    this.operatorController.leftBumper().whileTrue(this.roller.expellRoller());

    this.operatorController
        .leftJoystick()
        .toggleOnTrue(Commands.runOnce(() -> this.states.enableNoNOdometryShootState()))
        .toggleOnFalse(Commands.runOnce(() -> this.states.disableNoNOdometryShootState()));

    this.driverController.a().onTrue(this.turret.resetCRT());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
