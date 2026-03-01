package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.java_is_undercontrol.util.Util;
import frc.robot.constants.JoystickConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class DriverController implements IDriverController {

  private static DriverController mInstance = null;

  public static DriverController getInstance() {
    if (mInstance == null) {
      mInstance = new DriverController();
    }

    return mInstance;
  }

  final CommandXboxController driverController;

  private DriverController() {
    driverController = new CommandXboxController(0);
  }

  @Override
  @AutoLogOutput
  public double getLeftX() {
    return -driverController.getLeftX();
  }

  @Override
  @AutoLogOutput
  public double getLeftY() {
    return -driverController.getLeftY();
  }

  @Override
  public double getCOS_Joystick() {
    return -driverController.getRightX();
  }

  @Override
  public double getSIN_Joystick() {
    return -driverController.getRightY();
  }

  @Override
  public Trigger turboActivate() {
    return driverController.rightTrigger(0.2);
  }

  @Override
  public Trigger slowActivate() {
    return driverController.leftTrigger(0.2);
  }

  @Override
  public boolean notUsingJoystick() {
    return Util.inRange(
            getCOS_Joystick(),
            -JoystickConstants.DEADBAND_ROTATION,
            JoystickConstants.DEADBAND_ROTATION)
        && Util.inRange(
            getSIN_Joystick(),
            -JoystickConstants.DEADBAND_ROTATION,
            JoystickConstants.DEADBAND_ROTATION);
  }

  @Override
  @AutoLogOutput
  public boolean notUsingTranslationJoystick() {
    return Util.inRange(
            getLeftX(), -JoystickConstants.DEADBAND_ROTATION, JoystickConstants.DEADBAND_ROTATION)
        || Util.inRange(
            getLeftY(), -JoystickConstants.DEADBAND_ROTATION, JoystickConstants.DEADBAND_ROTATION);
  }

  @Override
  public Trigger a() {
    return driverController.a();
  }

  @Override
  public Trigger y() {
    return driverController.y();
  }

  @Override
  public Trigger x() {
    return driverController.x();
  }

  @Override
  public Trigger b() {
    return driverController.b();
  }

  @Override
  public Trigger leftBumper() {
    return driverController.leftBumper();
  }

  @Override
  public Trigger rightBumper() {
    return driverController.rightBumper();
  }

  @Override
  public Trigger isForcingDriverControl() {
    return driverController.leftTrigger(0.4);
  }

  @Override
  public Trigger start() {
    return driverController.start();
  }

  @Override
  public Trigger miniLeft() {
    return driverController.button(6);
  }

  @Override
  public Trigger leftArrow() {
    return driverController.povLeft();
  }

  @Override
  public Trigger rightArrow() {
    return driverController.povRight();
  }

  @Override
  public Trigger upArrow() {
    return driverController.povUp();
  }

  @Override
  public Trigger downArrow() {
    return driverController.povDown();
  }

  @Override
  public Trigger leftStick() {
    return driverController.leftStick();
  }

  @Override
  public Trigger rightStick() {
    return driverController.rightStick();
  }
}
