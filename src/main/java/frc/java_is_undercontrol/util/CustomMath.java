package frc.java_is_undercontrol.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class CustomMath {

  public static double toAbsoluteSpeed(ChassisSpeeds chassisSpeeds) {
    return Math.sqrt(
        Math.pow(chassisSpeeds.vxMetersPerSecond, 2)
            + Math.pow(chassisSpeeds.vyMetersPerSecond, 2));
  }
}
