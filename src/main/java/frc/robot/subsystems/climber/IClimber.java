package frc.robot.subsystems.climber;

public interface IClimber {

  void Climb();

  //void goToStowPosition();

  void periodic();

  void setCoastClimber();

  void setBrakeClimber();

  void startCollecting();

   boolean isClimbed();

    void defaultPosition();

}
