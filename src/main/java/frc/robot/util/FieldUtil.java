package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class FieldUtil {
 // private static final double FIELD_WIDTH_METERS = 8.02;

  // Grid Poses
  public static final double XPosition = 1.86;
  public static Pose2d Station1 =
      new Pose2d(new Translation2d(XPosition, 5.04), Rotation2d.fromDegrees(0));
  public static Pose2d Station2 =
      new Pose2d(new Translation2d(XPosition, 4.42), Rotation2d.fromDegrees(0));
  public static Pose2d Station3 =
      new Pose2d(new Translation2d(XPosition, 3.85), Rotation2d.fromDegrees(0));
  public static Pose2d Station4 =
      new Pose2d(new Translation2d(XPosition, 3.31), Rotation2d.fromDegrees(0));
  public static Pose2d Station5 =
      new Pose2d(new Translation2d(XPosition, 2.74), Rotation2d.fromDegrees(0));
  public static Pose2d Station6 =
      new Pose2d(new Translation2d(XPosition, 2.16), Rotation2d.fromDegrees(0));
  public static Pose2d Station7 =
      new Pose2d(new Translation2d(XPosition, 1.63), Rotation2d.fromDegrees(0));
  public static Pose2d Station8 =
      new Pose2d(new Translation2d(XPosition, 1.06), Rotation2d.fromDegrees(0));
  public static Pose2d Station9 =
      new Pose2d(new Translation2d(XPosition, 0.49), Rotation2d.fromDegrees(0));

  private FieldUtil() {
    // Utility class
  }

  public static Pose2d getTransformPoseStation(Pose2d station) {
    return station;
  /*   if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) return station;
    Translation2d transformedTranslation =
        new Translation2d(station.getX(), FIELD_WIDTH_METERS - station.getY());
    Rotation2d transformedHeading = station.getRotation().times(-1);
    return new Pose2d(transformedTranslation, transformedHeading);*/
  }
}
