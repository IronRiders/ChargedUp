package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class PreLevelingCommand extends CommandBase {
  private final double speed = 0.1;
  DriveSubsystem drive;

  private final double targetPitch = 3.0;

  public PreLevelingCommand(DriveSubsystem drive) {
    this.drive = drive;
  }

  public void execute() {
    drive.setChassisSpeeds(0, speed, 0, true);
  }

  @Override
  public boolean isFinished() {
    return drive.pigeon.getPitch() < targetPitch ? true : false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}