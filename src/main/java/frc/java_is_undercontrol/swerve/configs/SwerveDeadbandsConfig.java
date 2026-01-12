package frc.java_is_undercontrol.swerve.configs;

public class SwerveDeadbandsConfig {
  public double translationDeadband;

  public double joystickAimDeadband;

  public SwerveDeadbandsConfig(double translationDeadband, double joystickAimDeadband) {
    this.translationDeadband = translationDeadband;
    this.joystickAimDeadband = joystickAimDeadband;
  }
}
