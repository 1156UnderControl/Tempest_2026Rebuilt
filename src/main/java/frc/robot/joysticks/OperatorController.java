package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.java_is_undercontrol.util.Util;
import frc.robot.constants.JoystickConstants;

public class OperatorController implements IOperatorController {

  private static OperatorController mInstance = null;

  public static OperatorController getInstance() {
    if (mInstance == null) {
      mInstance = new OperatorController();
    }

    return mInstance;
  }

  final CommandXboxController OperatorController;

  private OperatorController() {
    OperatorController = new CommandXboxController(1);
  }

  @Override
  public double getLeftX() {
    return -OperatorController.getLeftX();
  }

  @Override
  public double getLeftY() {
    return -OperatorController.getLeftY();
  }

  @Override
  public double getCOS_Joystick() {
    return -OperatorController.getRightX();
  }

  @Override
  public double getSIN_Joystick() {
    return -OperatorController.getRightY();
  }

  @Override
  public Trigger rightTrigger() {
    return OperatorController.rightTrigger(0.2);
  }

  @Override
  public Trigger leftTrigger() {
    return OperatorController.leftTrigger(0.2);
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
  public Trigger a() {
    return OperatorController.a();
  }

  @Override
  public Trigger y() {
    return OperatorController.y();
  }

  @Override
  public Trigger x() {
    return OperatorController.x();
  }

  @Override
  public Trigger b() {
    return OperatorController.b();
  }

  @Override
  public Trigger leftBumper() {
    return OperatorController.leftBumper();
  }

  @Override
  public Trigger rightBumper() {
    return OperatorController.rightBumper();
  }

  @Override
  public Trigger isForcingDriverControl() {
    return OperatorController.leftTrigger(0.4);
  }

  @Override
  public Trigger start() {
    return OperatorController.start();
  }

  @Override
  public Trigger miniLeft() {
    return OperatorController.button(6);
  }

  @Override
  public Trigger leftArrow() {
    return OperatorController.povLeft();
  }

  @Override
  public Trigger rightArrow() {
    return OperatorController.povRight();
  }

  @Override
  public Trigger upArrow() {
    return OperatorController.povUp();
  }

  @Override
  public Trigger downArrow() {
    return OperatorController.povDown();
  }

  @Override
  public Trigger leftJoystick() {
    return OperatorController.leftStick();
  }

  @Override
  public Trigger rightJoystick() {
    return OperatorController.rightStick();
  }
}
