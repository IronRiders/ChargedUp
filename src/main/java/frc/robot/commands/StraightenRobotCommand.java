package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class StraightenRobotCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;

  public StraightenRobotCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    driveSubsystem.setChassisSpeeds(
        0,
        0,
        (driveSubsystem.pigeon.getRotation2d().getDegrees() % 360 > 180 ? -1 : 1)
            * Constants.STRAIGHTEN_ROBOT_TURN_SPEED,
        false);
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(driveSubsystem.pigeon.getRotation2d().getDegrees() % 360)
        < Constants.STRAIGHTEN_TALORANCE_ANGLE);
  }
}
