package frc.robot;

import com.ctre.phoenix6.controls.ModulateVBatOut;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.java_is_undercontrol.vision.Cameras.LimelightHelpers;
import frc.robot.constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.subsystems.shooter.ShooterCalculator;
import frc.robot.util.HubShiftUtil;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private LedSubsystem leds;

  CANdle candle = new CANdle(0, "light bus");
  SolidColor color = new SolidColor(0, 8);
StrobeAnimation strobeAnimation = new StrobeAnimation(0, 8);
  

  public Robot() {
    AutoLogOutputManager.addPackage("frc.java_is_undercontrol");
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    switch (Constants.getMode()) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/u/logs"));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();

    var modules =
        new SwerveModuleConstants[] {
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight
        };
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }

    robotContainer = new RobotContainer();
    leds = new LedSubsystem();
    LimelightHelpers.setLEDMode_ForceOff(VisionConstants.backRightLimelightName);
    LimelightHelpers.SetIMUMode(VisionConstants.backRightLimelightName, 0);
    LimelightHelpers.SetIMUAssistAlpha(VisionConstants.backRightLimelightName, 0.001);
    LimelightHelpers.SetThrottle(VisionConstants.backRightLimelightName, 200);

    RobotController.setBrownoutVoltage(5.9);
      
  }

  @Override
  public void robotPeriodic() {
    ShooterCalculator.getInstance().clearShootingParameters();
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("Is Active", HubShiftUtil.getOfficialShiftInfo().active());
    SmartDashboard.putString("Shift:", HubShiftUtil.getOfficialShiftInfo().currentShift().name());
    SmartDashboard.putNumber("Match Time:", HubShiftUtil.getOfficialShiftInfo().currentTime());
    SmartDashboard.putBoolean(
        "Won Auto",
        DriverStation.getAlliance().orElse(Alliance.Blue) == HubShiftUtil.getFirstActiveAlliance());
    SmartDashboard.putNumber(
        "Time Until Next Shift:", HubShiftUtil.getOfficialShiftInfo().remainingTime());

    leds.updateLEDs();
       
      }
          
  

  @Override
  public void disabledInit() {
    LimelightHelpers.setLEDMode_ForceOff(VisionConstants.backRightLimelightName);
    LimelightHelpers.SetIMUMode(VisionConstants.backRightLimelightName, 0);

   
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    LimelightHelpers.SetIMUMode(VisionConstants.backRightLimelightName, 0);
    LimelightHelpers.SetThrottle(VisionConstants.backRightLimelightName, 0);
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }

    RobotStates.getInstance().setShouldConfigRollerAuto(true);
    RobotStates.getInstance().setShouldConfigRollerTeleop(false);
  }

  @Override
  public void autonomousPeriodic() {
    if (RobotStates.getInstance().isShootingMode()) {
      LimelightHelpers.setLEDMode_ForceOn(VisionConstants.backRightLimelightName);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(VisionConstants.backRightLimelightName);
    }
  }

  @Override
  public void teleopInit() {
    HubShiftUtil.initialize();

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    LimelightHelpers.SetIMUMode(VisionConstants.backRightLimelightName, 0);
    LimelightHelpers.SetThrottle(VisionConstants.backRightLimelightName, 0);

    RobotStates.getInstance().setShouldConfigRollerAuto(false);
    RobotStates.getInstance().setShouldConfigRollerTeleop(true);
  }

  @Override
  public void teleopPeriodic() {


   if (HubShiftUtil.getOfficialShiftInfo().remainingTime() < 15 && HubShiftUtil.getOfficialShiftInfo().active())
     {
       candle.setControl(color);
    } else if
      (HubShiftUtil.getOfficialShiftInfo().active()){
           leds.updateLEDs();
     System.out.println("leds working");
       }
      
       
   }
      


     
  

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

}