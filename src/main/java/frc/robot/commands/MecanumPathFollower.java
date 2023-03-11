package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;

public class MecanumPathFollower extends CommandBase {

  private final DriveSubsystem drive;
  private String pathName;
  private PathConstraints constraints;
  private boolean resetOdom = false;
  private PathPlannerTrajectory trajectory;
  private PathPlannerTrajectory alliancePath;
  private CommandBase controllerCommand = Commands.none();

  // Pre-Planned Autonomous Pathing
  public MecanumPathFollower(
      DriveSubsystem drive, String pathName, PathConstraints constraints, boolean resetOdom) {
    this.drive = drive;
    this.pathName = pathName;
    this.constraints = constraints;
    this.resetOdom = resetOdom;

    addRequirements(drive);
  }

  // On the Fly Pathing
  public MecanumPathFollower(DriveSubsystem drive, PathPlannerTrajectory trajectory) {
    this.trajectory = trajectory;
    this.drive = drive;

    addRequirements(drive);
  }

  public void initialize() {
    if (pathName == null && trajectory == null) {
      end(false);
      return;
    } else if (pathName != null) {
      alliancePath = PathPlanner.loadPath(pathName, constraints);
      // var path = PathPlanner.loadPath(pathName, constraints);
      // alliancePath = PathPlannerTrajectory.transformTrajectoryForAlliance(path,
      // DriverStation.getAlliance());
    } else {
      alliancePath = trajectory;
    }
    if (resetOdom) drive.resetOdometry(alliancePath.getInitialHolonomicPose());

    controllerCommand =
        new PPMecanumControllerCommand(
            alliancePath,
            drive::getPose2d,
            drive.getxController(),
            drive.getyController(),
            drive.getThetaController(),
            (speed) -> drive.setChassisSpeeds(speed, true),
            drive);

    controllerCommand.initialize();
  }

  public void execute() {
    controllerCommand.execute();
  }

  public void end(boolean interrupted) {
    controllerCommand.end(interrupted);
  }

  public boolean isFinished() {
    return controllerCommand.isFinished();
  }
}
