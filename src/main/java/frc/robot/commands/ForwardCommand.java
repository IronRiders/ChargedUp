package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ForwardCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private double initialX;
  private double distance;

  public ForwardCommand(DriveSubsystem driveSubsystem, double distance) {
    this.driveSubsystem = driveSubsystem;
    this.distance = distance;

    addRequirements(driveSubsystem);
  }

  @Override()
  public void initialize() {
    initialX = driveSubsystem.getPose2d().getX();
    SmartDashboard.putNumber("Initial X", driveSubsystem.getPose2d().getX());
  }

  @Override
  public void execute() {
    driveSubsystem.setChassisSpeeds(0.0, -0.4, 0., false);
  }

  public boolean isFinished() {
    double positionDelta = Math.abs(driveSubsystem.getPose2d().getX() - initialX);
    return positionDelta >= distance;
  }
}
