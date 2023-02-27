package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
  public final PhotonCamera camera = new PhotonCamera("Anish");
  public AprilTagFieldLayout tagLayout;
  public PhotonPipelineResult previousPipelineResult = null;
  public Transform3d camToTarget = new Transform3d();
  public PhotonPoseEstimator photonPoseEstimator;

  public Vision() {
    try {
      tagLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      photonPoseEstimator =
          new PhotonPoseEstimator(
              tagLayout, PoseStrategy.MULTI_TAG_PNP, camera, Constants.RobotToCam);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
    }
  }

  public void periodic() {
    SmartDashboard.putNumber("Ground Distance to target", Units.metersToInches(estimateDistance()));
  }

  public double getYaw() {
    if (hasTarget()) return camera.getLatestResult().getBestTarget().getYaw();

    return 0;
  }

  public boolean hasTarget() {
    var result = camera.getLatestResult();
    return result.hasTargets();
  }

  public double estimateDistance() {
    if (hasTarget())
      return Units.metersToInches(new Pose3d().plus(camera.getLatestResult().getBestTarget().getBestCameraToTarget())
          .toPose2d().getTranslation().getNorm());
      return 0;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (photonPoseEstimator == null) return Optional.empty();
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  public Optional<Pose2d> tagLocalization(
      double xDistance, double yDistance, double angleRadians, Pose2d robotPos) {
    if (!hasTarget()) return Optional.empty();
    var target = camera.getLatestResult().getBestTarget();
    return Optional.of(
        new Pose3d(robotPos)
            .plus(Constants.RobotToCam)
            .plus(target.getBestCameraToTarget())
            .toPose2d()
            .plus(
                new Transform2d(
                    new Translation2d(
                        Units.inchesToMeters(xDistance), Units.inchesToMeters(yDistance)),
                    new Rotation2d(angleRadians))));
  }

  public Optional<Pose2d> fieldElementTracking(Pose2d robotPose) {
    double targetHeight =
        Units.inchesToMeters(4.5); // Change based on cone orientation and cubes game pieces
    if (!hasTarget()) return Optional.empty();
    var target = camera.getLatestResult().getBestTarget();
    Translation3d camToTargetTrl =
        frc.robot.util.PhotonUtils.estimateCamToTargetTrl(
            Constants.RobotToCam,
            targetHeight,
            Rotation2d.fromDegrees(target.getYaw()),
            Rotation2d.fromDegrees(-target.getPitch()));
    Translation3d robotToTargetTrl =
        new Pose3d(camToTargetTrl, new Rotation3d())
            .relativeTo(new Pose3d().plus(Constants.RobotToCam.inverse()))
            .getTranslation();
    Pose2d fieldPose =
        new Pose3d(robotPose).plus(new Transform3d(robotToTargetTrl, new Rotation3d())).toPose2d();
    return Optional.of(fieldPose);
  }
}
