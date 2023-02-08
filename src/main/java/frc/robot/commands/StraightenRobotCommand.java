package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.MecanumDrive;

public class StraightenRobotCommand extends CommandBase {
  private final MecanumDrive mecanumDrive;

  public StraightenRobotCommand(MecanumDrive mecanumDrive) {
    this.mecanumDrive = mecanumDrive;
    addRequirements(mecanumDrive);
  }

  @Override
  public void execute() {
    mecanumDrive.updateSpeed(
        0,
        0,
        (mecanumDrive.pigeon.getRotation2d().getDegrees() > 180 ? -1 : 1)
            * Constants.STRAIGHTEN_ROBOT_TURN_SPEED,
        false);
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(mecanumDrive.pigeon.getRotation2d().getDegrees())
        < Constants.STRAIGHTEN_TALORNCE_ANGLE);
  }
}
