package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.climber.ClimberConstants;
import frc.robot.constants.hopper.SpindexerConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class SpindexerSubsystem extends SubsystemBase {
  private static SpindexerSubsystem instance;

  public static SpindexerSubsystem getInstance() {
    if (instance == null) {
      instance = new SpindexerSubsystem();
    }
    return instance;
  }

  private final SparkFlex spindexerMotor =
      new SparkFlex(SpindexerConstants.ID_spindexerMotor, MotorType.kBrushless);
  private final SmartMotorController smartSpindexerMotor =
      new SparkWrapper(
          spindexerMotor, DCMotor.getNeoVortex(0), new SmartMotorControllerConfig()) {};

  private final DCMotor motors = DCMotor.getNeoVortex(1);

  private SmartMotorControllerConfig smartSpindexerMotorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              ClimberConstants.PID.kP,
              ClimberConstants.PID.kI,
              ClimberConstants.PID.kD,
              RPM.of(SpindexerConstants.SPINDEXER_MAX_VELOCITY),
              RotationsPerSecondPerSecond.of(SpindexerConstants.SPINDEXER_MAX_ACCELERATION))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(false)
          .withTelemetry("Spindexer Motor", TelemetryVerbosity.HIGH);

  private final FlyWheelConfig spindexerMechanismConfig =
      new FlyWheelConfig(smartSpindexerMotor).withDiameter(Inches.of(4)).withMass(Pounds.of(1));

  public SpindexerSubsystem() {}

  @Override
  public void periodic() {
    this.updateInputs();
  }

  @Override
  public void simulationPeriodic() {}

  public void updateInputs() {}
}
