package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class MatchStatusTracker {
  private static MatchStatusTracker instance;

  private InterpolatingDoubleTreeMap timeOfFlightMap;

  private Optional<Alliance> alliance = DriverStation.getAlliance();

  private SendableChooser<String> autoWinnerSimulated = new SendableChooser<>();

  public static MatchStatusTracker getInstance(InterpolatingDoubleTreeMap timeOfFlighMap) {
    if (instance == null) {
      return new MatchStatusTracker(timeOfFlighMap);
    }
    return instance;
  }

  public static MatchStatusTracker getInstance() {
    if (instance == null) {
      return new MatchStatusTracker(null);
    }
    return instance;
  }

  private MatchStatusTracker(InterpolatingDoubleTreeMap timeOfFlightMap) {
    this.timeOfFlightMap = timeOfFlightMap;
    this.autoWinnerSimulated.setDefaultOption("Red Auto Winner", "RED");
    this.autoWinnerSimulated.addOption("Blue Auto Winner", "BLUE");
    SmartDashboard.putData(autoWinnerSimulated);
  }

  public void sendMatchTimeToDashboard(){
    double matchTime = DriverStation.getMatchTime();
    SmartDashboard.putNumber("MatchTime", matchTime);
  }

  public void sendHubStatusToDashboard() {
    SmartDashboard.putBoolean("HubActive", isHubActive());
  }

  public boolean isHubActive(){
    return this.isHubActive(0);
  }


  public boolean isHubActive(double distanceFromHub) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return false;
    }
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }
    double matchTime = DriverStation.getMatchTime();
    if(this.timeOfFlightMap != null){
      matchTime = matchTime - this.timeOfFlightMap.get(distanceFromHub);
    }
    String gameData = DriverStation.getGameSpecificMessage();
    boolean redInactiveFirst = false;
    if (gameData.isEmpty() || DriverStation.isFMSAttached()) {
      switch (this.autoWinnerSimulated.getSelected()) {
        case "RED":
          redInactiveFirst = true;
          break;
        case "BLUE":
          redInactiveFirst = false;
        default:
          break;
      }
    }
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        return true;
      }
    }
    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    if (matchTime > 130) {
      // Transition shift
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active
      return true;
    }
  }
}
