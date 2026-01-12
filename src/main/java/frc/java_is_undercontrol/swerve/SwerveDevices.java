package frc.java_is_undercontrol.swerve;

import frc.java_is_undercontrol.swerve.io.gyro.GyroIO;
import frc.java_is_undercontrol.swerve.io.module.ModuleIO;

public class SwerveDevices {
  public GyroIO gyroIO;
  public ModuleIO flModuleIO;
  public ModuleIO frModuleIO;
  public ModuleIO blModuleIO;
  public ModuleIO brModuleIO;

  public SwerveDevices(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    this.flModuleIO = flModuleIO;
    this.frModuleIO = frModuleIO;
    this.blModuleIO = blModuleIO;
    this.brModuleIO = brModuleIO;
  }
}
