package frc.robot.joysticks;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.java_is_undercontrol.util.Util;
import frc.robot.constants.JoystickConstants;

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
  public double getLeftX() {
    return -driverController.getLeftX();
  }

  @Override
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
  public boolean notUsingJoystick() {
    return Util.inRange(getCOS_Joystick(), -JoystickConstants.DEADBAND, JoystickConstants.DEADBAND)
        && Util.inRange(getSIN_Joystick(), -JoystickConstants.DEADBAND, JoystickConstants.DEADBAND);
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
  public Trigger cancelAction() {
    return driverController.povLeft();
  }

  @Override
  public Trigger setHasAlgae() {
    return driverController.leftStick().and(driverController.rightStick().negate());
  }

  @Override
  public Trigger setHasCoral() {
    return driverController.rightStick().and(driverController.leftStick().negate());
  }

  @Override
  public Trigger setNoObject() {
    return driverController.leftStick().and(driverController.rightStick());
  }

  private double performAllianceInputDirectionCorrection(Double value) {
    Alliance alliance =
        DriverStation.getAlliance().isPresent()
            ? DriverStation.getAlliance().get()
            : DriverStation.Alliance.Red;
    if (alliance == Alliance.Red) {
      return -value;
    }
    return value;
  }
}
