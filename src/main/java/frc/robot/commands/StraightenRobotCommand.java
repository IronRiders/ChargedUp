package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class StraightenRobotCommand extends CommandBase {
  private final DriveSubsystem driveSubsytem;

  public StraightenRobotCommand(DriveSubsystem driveSubsytem) {
    this.driveSubsytem = driveSubsytem;

    addRequirements(driveSubsytem);
  }

  @Override
  public void execute() {
    isFinished();
    double angle = driveSubsytem.pigeon.getAngle();
    while (angle < 0) {
      angle += 360;
    }
    while (angle > 360) {
      angle -= 360;
    }
    driveSubsytem.setChassisSpeeds(
        0,
        0,
        (angle > 180
            ? -Constants.STRAIGHTEN_ROBOT_TURN_SPEED
            : Constants.STRAIGHTEN_ROBOT_TURN_SPEED),
        true);
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(driveSubsytem.pigeon.getAngle() % 360) < Constants.STRAIGHTEN_TALORANCE_ANGLE);
  }
}
