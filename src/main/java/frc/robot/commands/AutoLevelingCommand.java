package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLevelingCommand extends CommandBase {
  private final DriveSubsystem drive;
  private final WPI_Pigeon2 pigeon;

  public AutoLevelingCommand(DriveSubsystem drive) {
    this.drive = drive;
    pigeon = drive.pigeon;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    double forward = 0;

    if (Math.abs(pigeon.getPitch()) > 5) {
      forward = Constants.FORWARD_VELOCITY;
    }
  // if (Math.abs(pigeon.getRoll()) > 5) {
  //    strafe = Constants.FORWARD_VELOCITY;
 //   }
    if (pigeon.getPitch() < 5) {
      forward *= -1;
    }
 //   if (pigeon.getRoll() < 5) {
   //   strafe *= -1;
    //}
    drive.setChassisSpeeds(0, forward, 0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
