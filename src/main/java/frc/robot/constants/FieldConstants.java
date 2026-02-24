package frc.robot.constants;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import java.io.IOException;
import java.nio.file.Path;

public class FieldConstants {
  public static final double fieldLength = 16.518;
  public static final double fieldWidth = 8.043;

  public static final int aprilTagCount = AprilTagLayoutType.OFFICIAL.layout.getTags().size();
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  public enum AprilTagLayoutType {
    OFFICIAL("2026-official"),
    NONE("2026-none");

    AprilTagLayoutType(String name) {
      try {
        layout =
            new AprilTagFieldLayout(
                Constants.disableHAL
                    ? Path.of("src", "main", "deploy", "apriltags", name + ".json")
                    : Path.of(
                        Filesystem.getDeployDirectory().getPath(), "apriltags", name + ".json"));
      } catch (IOException e) {
        throw new RuntimeException(e);
      }

      try {
        layoutString = new ObjectMapper().writeValueAsString(layout);
      } catch (JsonProcessingException e) {
        throw new RuntimeException(
            "Failed to serialize AprilTag layout JSON " + toString() + "for Northstar");
      }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;
  }

  public class FieldStructurePoses {
    public static final Translation2d BLUE_HUB_POSE = new Translation2d(4.626, 4.035);
    public static final Translation2d RED_HUB_POSE = new Translation2d(11.915, 4.035);

    public static final Pose2d BLUE_LEFT_TOWER = new Pose2d(1.113, 4.611, Rotation2d.fromDegrees(90));
    public static final Pose2d BLUE_RIGHT_TOWER =
        new Pose2d(BLUE_LEFT_TOWER.getX(), 2.881, Rotation2d.fromDegrees(-90));

    public static final Pose2d RED_LEFT_TOWER =
        new Pose2d(
            16.541052 - BLUE_LEFT_TOWER.getX(),
            8.069263 - BLUE_LEFT_TOWER.getY(),
            Rotation2d.fromDegrees(-90));
    public static final Pose2d RED_RIGHT_TOWER =
        new Pose2d(
            16.541052 - BLUE_RIGHT_TOWER.getX(),
            8.069263 - BLUE_RIGHT_TOWER.getY(),
            Rotation2d.fromDegrees(90));

    public static final Pose2d SAFE_ALLIANCE_TRENCH_POSE_LEFT_BLUE =
        new Pose2d(3.220, 7.422, Rotation2d.fromDegrees(0));
    public static final Pose2d SAFE_ALLIANCE_TRENCH_POSE_RIGHT_BLUE =
        new Pose2d(
            3.220,
            FieldConstants.fieldWidth - SAFE_ALLIANCE_TRENCH_POSE_LEFT_BLUE.getY(),
            Rotation2d.fromDegrees(0));

    public static final Pose2d SAFE_ALLIANCE_TRENCH_POSE_LEFT_RED =
        new Pose2d(
            FieldConstants.fieldLength - SAFE_ALLIANCE_TRENCH_POSE_LEFT_BLUE.getX(),
            SAFE_ALLIANCE_TRENCH_POSE_RIGHT_BLUE.getY(),
            Rotation2d.fromDegrees(180));
    public static final Pose2d SAFE_ALLIANCE_TRENCH_POSE_RIGHT_RED =
        new Pose2d(
            FieldConstants.fieldLength - SAFE_ALLIANCE_TRENCH_POSE_LEFT_BLUE.getX(),
            SAFE_ALLIANCE_TRENCH_POSE_LEFT_BLUE.getY(),
            Rotation2d.fromDegrees(180));

    public static final Pose2d SAFE_MIDDLE_TRENCH_POSE_LEFT_BLUE =
        new Pose2d(5.851, 7.422, Rotation2d.fromDegrees(0));
    public static final Pose2d SAFE_MIDDLE_TRENCH_POSE_RIGHT_BLUE =
        new Pose2d(
            5.851,
            FieldConstants.fieldWidth - SAFE_MIDDLE_TRENCH_POSE_LEFT_BLUE.getY(),
            Rotation2d.fromDegrees(0));

    public static final Pose2d SAFE_MIDDLE_TRENCH_POSE_LEFT_RED =
        new Pose2d(
            FieldConstants.fieldLength - SAFE_MIDDLE_TRENCH_POSE_LEFT_BLUE.getX(),
            SAFE_MIDDLE_TRENCH_POSE_RIGHT_BLUE.getY(),
            Rotation2d.fromDegrees(180));
    public static final Pose2d SAFE_MIDDLE_TRENCH_POSE_RIGHT_RED =
        new Pose2d(
            FieldConstants.fieldLength - SAFE_MIDDLE_TRENCH_POSE_LEFT_BLUE.getX(),
            SAFE_MIDDLE_TRENCH_POSE_LEFT_BLUE.getY(),
            Rotation2d.fromDegrees(180));
  }

  public class FieldStrategicPoses {
    public static final Translation2d RIGHT_FEED_POSE =
        new Translation2d(FieldConstants.fieldLength / 8, (FieldConstants.fieldWidth / 4));
    public static final Translation2d LEFT_FEED_POSE =
        new Translation2d(
            RIGHT_FEED_POSE.getX(), FieldConstants.fieldWidth - RIGHT_FEED_POSE.getY());

    public static final double AllianceZoneX = 4.007866 + (TunerConstants.ROBOT_SIZE / 2);
    public static final double OtherAllianceZoneX =
        (fieldLength - 4.007866) + (TunerConstants.ROBOT_SIZE / 2);
    public static final double ShootZoneX = 7.669;

    public static final double NOT_SHOOT_LEFT_Y = 4.6;
    public static final double NOT_SHOOT_RIGHT_Y = 3.45;
  }
}
