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

  private SendableChooser<String> autoWinnerSimulated = new SendableChooser<>();

  public static MatchStatusTracker getInstance(InterpolatingDoubleTreeMap timeOfFlighMap) {
    if (instance == null) {
      instance = new MatchStatusTracker(timeOfFlighMap);
    }
    return instance;
  }

  public static MatchStatusTracker getInstance() {
    if (instance == null) {
      instance = new MatchStatusTracker(null);
    }
    return instance;
  }

  private MatchStatusTracker(InterpolatingDoubleTreeMap timeOfFlightMap) {
    this.timeOfFlightMap = timeOfFlightMap;
    this.autoWinnerSimulated.setDefaultOption("Red Auto Winner", "RED");
    this.autoWinnerSimulated.addOption("Blue Auto Winner", "BLUE");
    SmartDashboard.putData("AutoWinnerSimulated", autoWinnerSimulated);
  }

  public void sendMatchTimeToDashboard() {
    double matchTime = DriverStation.getMatchTime();
    SmartDashboard.putNumber("MatchTime", matchTime);
  }

  public void sendHubStatusToDashboard() {
    SmartDashboard.putBoolean("HubActive", isHubActive());
  }

  public void sendHubTiming() {
    double timeToSend;
    if (getHubActivationInfo().hubActive()) {
      timeToSend = getHubActivationInfo().timeLeftWhileActive();
    } else {
      timeToSend = getHubActivationInfo().timeUntilHubActive();
    }
    SmartDashboard.putNumber("TimeOfHub", timeToSend);
  }

  public boolean isHubActive() {
    return this.isHubActive(0);
  }

  public boolean isHubActive(double distanceFromHub) {
    return getHubActivationInfo(distanceFromHub).hubActive();
  }

  public HubActivationInfo getHubActivationInfo() {
    return getHubActivationInfo(0);
  }

  public HubActivationInfo getHubActivationInfo(double distanceFromHub) {
    double matchTime = getAdjustedMatchTime(distanceFromHub);
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return new HubActivationInfo(false, Double.POSITIVE_INFINITY, 0.0);
    }
    if (DriverStation.isAutonomousEnabled()) {
      return new HubActivationInfo(true, 0.0, matchTime);
    }
    if (!DriverStation.isTeleopEnabled()) {
      return new HubActivationInfo(false, Double.POSITIVE_INFINITY, 0.0);
    }
    Optional<Boolean> redInactiveFirst = getRedInactiveFirst();
    if (redInactiveFirst.isEmpty()) {
      return new HubActivationInfo(true, 0.0, matchTime);
    }
    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> !redInactiveFirst.get();
          case Blue -> redInactiveFirst.get();
        };
    HubPeriod currentPeriod = HubPeriod.fromMatchTime(matchTime);
    boolean hubActive = currentPeriod.isActive(shift1Active);
    double timeUntilActive =
        hubActive ? 0.0 : timeUntilHubBecomesActive(currentPeriod, matchTime, shift1Active);
    double timeLeftActive =
        hubActive ? timeUntilHubBecomesInactive(currentPeriod, matchTime, shift1Active) : 0.0;
    return new HubActivationInfo(hubActive, timeUntilActive, timeLeftActive);
  }

  private Optional<Boolean> getRedInactiveFirst() {
    String gameData = DriverStation.getGameSpecificMessage();
    boolean redInactiveFirst = false;
    if (gameData.isEmpty() || !DriverStation.isFMSAttached()) {
      String selected = this.autoWinnerSimulated.getSelected();
      if (selected == null || selected.equals("RED")) {
        redInactiveFirst = true;
      } else if (selected.equals("BLUE")) {
        redInactiveFirst = false;
      }
    }
    if (gameData.isEmpty()) {
      return Optional.of(redInactiveFirst);
    }
    return switch (gameData.charAt(0)) {
      case 'R' -> Optional.of(true);
      case 'B' -> Optional.of(false);
      default -> Optional.empty();
    };
  }

  private double getAdjustedMatchTime(double distanceFromHub) {
    double matchTime = DriverStation.getMatchTime();
    if (Double.isNaN(matchTime)) {
      matchTime = 0.0;
    }
    if (this.timeOfFlightMap != null) {
      matchTime -= this.timeOfFlightMap.get(distanceFromHub);
    }
    return Math.max(matchTime, 0.0);
  }

  private double timeUntilHubBecomesActive(
      HubPeriod currentPeriod, double matchTime, boolean shift1Active) {
    if (currentPeriod.isActive(shift1Active)) {
      return 0.0;
    }
    double waitTime = 0.0;
    double cursor = matchTime;
    HubPeriod[] periods = HubPeriod.values();
    for (int i = currentPeriod.ordinal(); i < periods.length; i++) {
      HubPeriod period = periods[i];
      if (i == currentPeriod.ordinal()) {
        waitTime += Math.max(0.0, cursor - period.endTime());
        cursor = period.endTime();
        continue;
      }
      if (period.isActive(shift1Active)) {
        return waitTime;
      }
      waitTime += Math.max(0.0, cursor - period.endTime());
      cursor = period.endTime();
    }
    return waitTime;
  }

  private double timeUntilHubBecomesInactive(
      HubPeriod currentPeriod, double matchTime, boolean shift1Active) {
    if (!currentPeriod.isActive(shift1Active)) {
      return 0.0;
    }
    double activeTime = 0.0;
    double cursor = matchTime;
    HubPeriod[] periods = HubPeriod.values();
    for (int i = currentPeriod.ordinal(); i < periods.length; i++) {
      HubPeriod period = periods[i];
      double periodDuration = Math.max(0.0, cursor - period.endTime());
      if (i == currentPeriod.ordinal()) {
        activeTime += periodDuration;
        cursor = period.endTime();
        continue;
      }
      if (!period.isActive(shift1Active)) {
        return activeTime;
      }
      activeTime += periodDuration;
      cursor = period.endTime();
    }
    return activeTime;
  }

  public record HubActivationInfo(
      boolean hubActive, double timeUntilHubActive, double timeLeftWhileActive) {}

  private enum HubPeriod {
    TRANSITION(130.0),
    SHIFT1(105.0),
    SHIFT2(80.0),
    SHIFT3(55.0),
    SHIFT4(30.0),
    ENDGAME(0.0);

    private final double endTime;

    HubPeriod(double endTime) {
      this.endTime = endTime;
    }

    static HubPeriod fromMatchTime(double matchTime) {
      if (matchTime > 130.0) {
        return TRANSITION;
      } else if (matchTime > 105.0) {
        return SHIFT1;
      } else if (matchTime > 80.0) {
        return SHIFT2;
      } else if (matchTime > 55.0) {
        return SHIFT3;
      } else if (matchTime > 30.0) {
        return SHIFT4;
      }
      return ENDGAME;
    }

    boolean isActive(boolean shift1Active) {
      return switch (this) {
        case TRANSITION, ENDGAME -> true;
        case SHIFT1, SHIFT3 -> shift1Active;
        case SHIFT2, SHIFT4 -> !shift1Active;
      };
    }

    double endTime() {
      return endTime;
    }
  }
}
