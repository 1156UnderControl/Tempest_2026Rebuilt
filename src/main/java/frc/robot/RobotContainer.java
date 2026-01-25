// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.java_is_undercontrol.swerve.SwerveDevices;
import frc.java_is_undercontrol.swerve.io.gyro.GyroIO;
import frc.java_is_undercontrol.swerve.io.gyro.GyroIOPigeon2;
import frc.java_is_undercontrol.swerve.io.module.ModuleIO;
import frc.java_is_undercontrol.swerve.io.module.ModuleIOSim;
import frc.java_is_undercontrol.swerve.io.module.ModuleIOTalonFX;
import frc.robot.generated.TunerConstants;
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.IDriverController;
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

  private final SwerveSubsystem drive;

  private final States states;

  private final TurretSubsystem turret;

  private final HoodSubsystem hood;

  private final FlyWheelSubsystem flyWheel;

  public final IDriverController driverController;

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

    this.turret = TurretSubsystem.getInstance();
    this.hood = HoodSubsystem.getInstance();
    this.flyWheel = FlyWheelSubsystem.getInstance();

    this.namedCommandsRegistry = new NamedCommandsRegistery(drive);
    this.namedCommandsRegistry.registerAllAutoCommands();

    this.states = new States(this.drive, this.turret, this.hood, this.flyWheel);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption("wheelRadiusCharacterization", drive.wheelRadiusCharacterization());
    autoChooser.addOption("feedForwardCharacterization", drive.feedforwardCharacterization());

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    this.states.setDefaultStates();
    this.driverController.a().onTrue(this.states.shootState());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
