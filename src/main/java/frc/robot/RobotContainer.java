// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.java_is_undercontrol.swerve.SwerveDevices;
import frc.java_is_undercontrol.swerve.io.gyro.GyroIO;
import frc.java_is_undercontrol.swerve.io.gyro.GyroIOPigeon2;
import frc.java_is_undercontrol.swerve.io.module.ModuleIO;
import frc.java_is_undercontrol.swerve.io.module.ModuleIOSim;
import frc.java_is_undercontrol.swerve.io.module.ModuleIOTalonFX;
import frc.robot.commands.swerve.GoToClimbPoseLeft;
import frc.robot.generated.TunerConstants;
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.IDriverController;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.swerve.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final SwerveSubsystem drive;

  private final ClimberSubsystem climber;

  public final IDriverController driverController;
  
  private final NamedCommandsRegistery namedCommandsRegistry;

  private final States states;

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

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    this.driverController = DriverController.getInstance();

    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new SwerveSubsystem(talonFxSwervedevices);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new SwerveSubsystem(simulatedSwerveDevices);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new SwerveSubsystem(replayedDevices);
        break;
    }

    climber = new ClimberSubsystem();
    this.namedCommandsRegistry = new NamedCommandsRegistery(drive);
    this.namedCommandsRegistry.registerAllAutoCommands();

    this.states = new States(drive);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption("wheelRadiusCharacterization", drive.wheelRadiusCharacterization());
    autoChooser.addOption("feedForwardCharacterization", drive.feedforwardCharacterization());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(Commands.run(() -> drive.driveAlignAngleJoystick(), drive));
    climber.setDefaultCommand(climber.setDutyCycle(0));
    driverController.rightArrow().whileTrue(Commands.run(() -> drive.goToClimbPoseRight(), drive));
    driverController.leftArrow().whileTrue(Commands.run(() -> drive.goToClimbPoseLeft(), drive));
    driverController.a().whileTrue(climber.setDutyCycle(0.2));
    driverController.b().whileTrue(climber.setDutyCycle(-0.2));
    this.states.setDefaultStates();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
