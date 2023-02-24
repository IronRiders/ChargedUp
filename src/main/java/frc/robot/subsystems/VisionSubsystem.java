package frc.robot.subsystems;

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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
  public final PhotonCamera camera = new PhotonCamera("Anish");
  public AprilTagFieldLayout tagLayout;
  public PhotonPipelineResult previousPipelineResult = null;
  public Transform3d camToTarget = new Transform3d();
  public PhotonPoseEstimator photonPoseEstimator;

  public VisionSubsystem() {
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
      Units.metersToInches(new Pose3d().plus(camToTarget).toPose2d().getTranslation().getNorm());
    return 0;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (photonPoseEstimator == null) return Optional.empty();
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }
}
