package frc.java_is_undercontrol.control;

public class ProfiledPIDConfig {

  public final double kP;

  public final double kI;

  public final double kD;

  public final double kF;

  public final double iZone;

  public final double maxVelocity;

  public final double maxAcceleration;

  public ProfiledPIDConfig(
      double kP,
      double kI,
      double kD,
      double kF,
      double iZone,
      double maxVelocity,
      double maxAcceleration) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kF = kF;
    this.iZone = iZone;
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
  }

  public ProfiledPIDConfig(
      double kP, double kI, double kD, double kF, double maxVelocity, double maxAcceleration) {
    this(kP, kI, kD, kF, 0, maxVelocity, maxAcceleration);
  }

  public ProfiledPIDConfig(
      double kP, double kI, double kD, double maxVelocity, double maxAcceleration) {
    this(kP, kI, kD, 0, maxVelocity, maxAcceleration);
  }

  public ProfiledPIDConfig(double kP, double kD, double maxVelocity, double maxAcceleration) {
    this(kP, 0, kD, 0, maxVelocity, maxAcceleration);
  }
}
