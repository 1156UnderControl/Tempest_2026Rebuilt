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
  public Trigger r2() {
    return new Trigger(() -> OperatorController.getR2Axis() > 0.2);
  }

  @Override
  public Trigger l2() {
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
  public Trigger cross() {
    return OperatorController.cross();
  }

  @Override
  public Trigger triangle() {
    return OperatorController.triangle();
  }

  @Override
  public Trigger square() {
    return OperatorController.square();
  }

  @Override
  public Trigger circle() {
    return OperatorController.circle();
  }

  @Override
  public Trigger l1() {
    return OperatorController.L1();
  }

  @Override
  public Trigger r1() {
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
  public Trigger l3() {
    return OperatorController.L3();
  }

  @Override
  public Trigger r3() {
    return OperatorController.R3();
  }
}
