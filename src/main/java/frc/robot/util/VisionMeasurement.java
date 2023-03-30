package frc.robot.util;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionMeasurement {
    public final PhotonTrackedTarget target;
    public final Pose2d robotPose2d;
    public final double timestampSeconds;
    public final double ambiguity;

    public VisionMeasurement(PhotonTrackedTarget target, Pose2d robotPose2d, double timestampSeconds, double ambiguity) {
        this.target = target;
        this.robotPose2d = robotPose2d;
        this.timestampSeconds = timestampSeconds;
        this.ambiguity = ambiguity;
    }
    
}
