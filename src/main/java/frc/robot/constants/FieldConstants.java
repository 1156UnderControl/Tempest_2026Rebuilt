package frc.robot.constants;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import java.io.IOException;
import java.nio.file.Path;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final double fieldLength = AprilTagLayoutType.OFFICIAL.layout.getFieldLength();
  public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.layout.getFieldLength();

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
    public static final Pose2d BLUE_HUB_POSE = new Pose2d(4.626, 4.035, new Rotation2d());
    public static final Pose2d RED_HUB_POSE = new Pose2d(11.915, 4.035, new Rotation2d());

    public static final Pose2d BLUE_LEFT_TOWER = new Pose2d(1.227, 4.611, new Rotation2d());
    public static final Pose2d BLUE_RIGHT_TOWER =
        new Pose2d(0.906, 2.857, Rotation2d.fromDegrees(180));

    public static final Pose2d RED_LEFT_TOWER = new Pose2d(16.541052-BLUE_LEFT_TOWER.getX(), 8.069263-BLUE_LEFT_TOWER.getY(), new Rotation2d());
    public static final Pose2d RED_RIGHT_TOWER =
        new Pose2d(16.541052-BLUE_RIGHT_TOWER.getX(), 8.069263-BLUE_RIGHT_TOWER.getY(), Rotation2d.fromDegrees(180));
  }
}