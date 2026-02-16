package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
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

  final CommandPS5Controller OperatorController;

  private OperatorController() {
    OperatorController = new CommandPS5Controller(1);
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
    return new Trigger(() -> OperatorController.getR2Axis() > 0.2);
  }

  @Override
  public Trigger leftTrigger() {
    return new Trigger(() -> OperatorController.getL2Axis() > 0.2);
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
    return OperatorController.cross();
  }

  @Override
  public Trigger y() {
    return OperatorController.triangle();
  }

  @Override
  public Trigger x() {
    return OperatorController.square();
  }

  @Override
  public Trigger b() {
    return OperatorController.circle();
  }

  @Override
  public Trigger leftBumper() {
    return OperatorController.L1();
  }

  @Override
  public Trigger rightBumper() {
    return OperatorController.R1();
  }

  @Override
  public Trigger isForcingDriverControl() {
    return new Trigger(() -> OperatorController.getL2Axis() > 0.4);
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
    return OperatorController.L3();
  }

  @Override
  public Trigger rightJoystick() {
    return OperatorController.R3();
  }
}
