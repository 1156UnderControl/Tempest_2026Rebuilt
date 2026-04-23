package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.ModulateVBatOut;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.java_is_undercontrol.vision.Cameras.LimelightHelpers;
import frc.robot.constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.shooter.ShooterCalculator;
import frc.robot.util.HubShiftUtil;

import static edu.wpi.first.units.Units.Degrees;

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

   private static final RGBWColor kGreen = new RGBWColor(0, 217, 0, 0);
    private static final RGBWColor kWhite = new RGBWColor(Color.kWhite).scaleBrightness(0.5);
    private static final RGBWColor kViolet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
    private static final RGBWColor kRed = new RGBWColor(217, 0, 0, 0);
    private static final RGBWColor kBlue = new RGBWColor(0, 0, 217, 0);


    private final CANdle m_candle = new CANdle(0, CANBus.roboRIO());

    private enum AnimationType {
        None,
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        Strobe,
        Twinkle,
        TwinkleOff,
    }

    private AnimationType m_anim0State = AnimationType.ColorFlow;
    private final SendableChooser<AnimationType> m_anim0Chooser = new SendableChooser<AnimationType>();
  

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

     var cfg = new CANdleConfiguration();
        cfg.LED.BrightnessScalar = 0.5;
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        m_candle.getConfigurator().apply(cfg);

   
        for (int i = 0; i < 8; ++i) {
            m_candle.setControl(new EmptyAnimation(i));
        }
        m_anim0Chooser.setDefaultOption("Rainbow", AnimationType.Rainbow);
        m_anim0Chooser.addOption("Color Flow", AnimationType.ColorFlow);
        m_anim0Chooser.addOption("Twinkle", AnimationType.Twinkle);
        m_anim0Chooser.addOption("Twinkle Off", AnimationType.TwinkleOff);
        m_anim0Chooser.addOption("Fire", AnimationType.Fire);

    SmartDashboard.putData("Animation 0", m_anim0Chooser);
      
    robotContainer = new RobotContainer();
   
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

final var anim0Selection = m_anim0Chooser.getSelected();
if (m_anim0State != anim0Selection) {
    m_anim0State = anim0Selection; 
            switch (m_anim0State) {
                default:
                case ColorFlow:
                    m_candle.setControl(
                        new ColorFlowAnimation(0, 7).withSlot(0)
                            .withColor(kViolet)
                    );
                    break;
                case Rainbow:
                    m_candle.setControl(
                        new RainbowAnimation(0, 7).withSlot(0)
                    );
                    break;
                case Twinkle:
                    m_candle.setControl(
                        new TwinkleAnimation(0, 7).withSlot(0)
                            .withColor(kWhite)
                    );
                    break;
                case TwinkleOff:
                    m_candle.setControl(
                        new TwinkleOffAnimation(0, 7).withSlot(0)
                            .withColor(kRed)
                    );
                    break;
                case Fire:
                    m_candle.setControl(
                        new FireAnimation(0, 7).withSlot(0)
                    );
                    break;
                  case Larson:
                    m_candle.setControl(
                        new LarsonAnimation(0, 7).withSlot(0)
                        .withColor(kBlue)
                    );
                    break;
            }}
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
      m_candle.setControl(
                        new FireAnimation(0, 7).withSlot(0));
    } else {
      m_candle.setControl(
                        new RainbowAnimation(0, 7).withSlot(0));
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
     m_candle.setControl(
                        new StrobeAnimation(0, 7).withSlot(0)
                            .withColor(kGreen));
    } else if
     (HubShiftUtil.getOfficialShiftInfo().active()){
     m_candle.setControl(
                         new ColorFlowAnimation(0, 7).withSlot(0)
                            .withColor(kGreen));
       } else {
 m_candle.setControl(
                        new RainbowAnimation(0, 7).withSlot(0));
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