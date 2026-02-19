package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IOperatorController {

  double getLeftX();

  double getLeftY();

  double getCOS_Joystick();

  double getSIN_Joystick();

  Trigger r2();

  Trigger l2();

  boolean notUsingJoystick();

  Trigger triangle();

  Trigger circle();

  Trigger cross();

  Trigger square();

  Trigger l1();

  Trigger r1();

  Trigger isForcingDriverControl();

  Trigger leftArrow();

  Trigger rightArrow();

  Trigger upArrow();

  Trigger downArrow();

  Trigger l3();

  Trigger r3();
}
