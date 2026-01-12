package frc.java_is_undercontrol.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface SensorIO {

  @AutoLog
  public static class SensorIOInputs {
    public boolean connected = true;
    public boolean value = false;
    public boolean inverted = false;
    public double timestampSec = 0.0;
  }

  void updateInputs(SensorIOInputs inputs);

  boolean getBoolean();

  int getBinary();
}
