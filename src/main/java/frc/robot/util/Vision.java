package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Vision {

  public static Vision instance;
  public final NetworkTableEntry rawBytesEntry;
  private double lastUpdateTimeMicro;
  private ArrayList<VisionMeasurement> measurements = new ArrayList<>();
  private PhotonTrackedTarget bestTarget;
  private VisionMeasurement bestMeasurement;
  public final PhotonCamera limelight = new PhotonCamera("Anish");
  public final PhotonCamera piCam = new PhotonCamera("CalvinAnish");

  public static AprilTagFieldLayout tagLayout = null;
  public PhotonPipelineResult previousPipelineResult = null;
  public Transform3d camToTarget = new Transform3d();
  public PhotonPoseEstimator photonPoseEstimator;

  public Vision() {
    NetworkTableInstance nInstance = NetworkTableInstance.getDefault();
    rawBytesEntry = nInstance.getTable("photonvision").getSubTable("camera.getName").getEntry("rawBytes");
    try {
      tagLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      photonPoseEstimator = new PhotonPoseEstimator(
          tagLayout, PoseStrategy.MULTI_TAG_PNP, piCam, Constants.RobotToCam); // Change Camera here too
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    } catch (Exception E) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", E.getStackTrace());
      photonPoseEstimator = null;
    }

  }

  public static Vision getVision() {
    if (instance == null) {
      instance = new Vision();
    }
    return instance;
  }

  public List<VisionMeasurement> getMeasurement() {
    update();
    return measurements;
  }

  public PhotonTrackedTarget getBestTarget() {
    update();
    return bestTarget;
  }

  public VisionMeasurement getBestMeasurement() {
    update();
    return bestMeasurement;
  }

  public boolean hasTarget(PhotonCamera cam) {
    var result = cam.getLatestResult();
    return result.hasTargets();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (photonPoseEstimator == null)
      return Optional.empty();
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  public void update() {
    if (lastUpdateTimeMicro == rawBytesEntry.getLastChange()) {
      SmartDashboard.putBoolean("No last time updated", true);
      return;
    } else {
      lastUpdateTimeMicro = rawBytesEntry.getLastChange();
      SmartDashboard.putBoolean("No last Time Updated", false);
    }

    previousPipelineResult = limelight.getLatestResult(); // CHANGE THIS TO DIFFERENT CAMERA
    if (!previousPipelineResult.hasTargets()) {
      SmartDashboard.putBoolean("No Targets", true);
      measurements.clear();
      bestMeasurement = null;
      bestTarget = null;
      return;
    }
    SmartDashboard.putBoolean("No Targets", false);
    bestTarget = previousPipelineResult.getBestTarget();
    Pose3d bestPose3d = getRobotPoseFromTarget(bestTarget);
    SmartDashboard.putBoolean("Best Target", bestTarget == null);

    if (bestPose3d == null) {
      bestMeasurement = null;
    } else {
      bestMeasurement = new VisionMeasurement(bestTarget,
          new Pose2d(bestPose3d.getX(), bestPose3d.getY(), new Rotation2d(bestPose3d.getRotation().getZ())),
          previousPipelineResult.getTimestampSeconds(),
          bestTarget.getPoseAmbiguity());
    }

    measurements.clear();

    for (PhotonTrackedTarget target : previousPipelineResult.targets) {
      Pose3d estRobPose3d = getRobotPoseFromTarget(target);
      if (estRobPose3d == null)
        continue;

      measurements.add(new VisionMeasurement(target,
          new Pose2d(estRobPose3d.getX(), estRobPose3d.getY(), new Rotation2d(estRobPose3d.getRotation().getZ())),
          previousPipelineResult.getTimestampSeconds(),
          target.getPoseAmbiguity()));
    }
  }

  public static Pose3d getRobotPoseFromTarget(PhotonTrackedTarget target) {
    Transform3d cameraToTarget = target.getBestCameraToTarget();

    Optional<Pose3d> feducialPose = tagLayout.getTagPose(target.getFiducialId());

    if (feducialPose.isEmpty()) {
      return null;
    }
    return PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, feducialPose.get(), Constants.RobotTopiCam);
  }

  public Optional<Pose2d> tagLocalization(
      double xDistance, double yDistance, double angleRadians, Pose2d robotPos,
      PhotonCamera cam) {
    if (!hasTarget(cam))
      return Optional.empty();
    var target = cam.getLatestResult().getBestTarget();
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

  public Optional<Pose2d> fieldElementTracking(Pose2d robotPose, PhotonCamera cam) {
    double targetHeight = Units.inchesToMeters(4.5); // Change based on cone orientation and cubes game
    if (!hasTarget(cam))
      return Optional.empty();
    var target = cam.getLatestResult().getBestTarget();
    Translation3d camToTargetTrl = frc.robot.util.PhotonUtils.estimateCamToTargetTrl(
        Constants.RobotToCam,
        targetHeight,
        Rotation2d.fromDegrees(target.getYaw()),
        Rotation2d.fromDegrees(-target.getPitch()));
    Translation3d robotToTargetTrl = new Pose3d(camToTargetTrl, new Rotation3d())
        .relativeTo(new Pose3d().plus(Constants.RobotToCam.inverse()))
        .getTranslation();
    Pose2d fieldPose = new Pose3d(robotPose).plus(new Transform3d(robotToTargetTrl, new Rotation3d())).toPose2d();
    return Optional.of(fieldPose);
  }
}