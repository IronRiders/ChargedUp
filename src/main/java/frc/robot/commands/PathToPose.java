package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class PathToPose extends CommandBase {

  PathPlannerTrajectory trajectory;
  CommandBase mecanumPathFollower = Commands.none();
  private final DriveSubsystem drive;
  private final Supplier<Pose2d> targetPoseSupplier;

  // Pose2d targetPose
  public PathToPose(DriveSubsystem drive, Supplier<Pose2d> targetPoseSupplier) {
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d robotPose = drive.getPose2d();
    Pose2d targetPose = targetPoseSupplier.get();
    if (targetPose == null) {
      end(false);
      return;
    }
    PathPoint initialPoint =
        new PathPoint(
            robotPose.getTranslation(), new Rotation2d(0), drive.getPose2d().getRotation());
    PathPoint finalPoint =
        new PathPoint(
            targetPose.getTranslation(),
            new Rotation2d(0),
            new Rotation2d(Units.degreesToRadians(180)));

    trajectory = PathPlanner.generatePath(Constants.SlowAutoConstraints, initialPoint, finalPoint);

    mecanumPathFollower = new MecanumPathFollower(drive, trajectory);
    mecanumPathFollower.initialize();
  }

  @Override
  public void execute() {
    mecanumPathFollower.execute();
  }

  @Override
  public void end(boolean interrupted) {
    mecanumPathFollower.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return mecanumPathFollower.isFinished();
  }
}
