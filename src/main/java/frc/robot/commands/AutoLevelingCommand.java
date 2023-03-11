package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLevelingCommand extends CommandBase {
  private final DriveSubsystem drive;
  private final WPI_Pigeon2 pigeon;
  private double[] gyroData;

  public AutoLevelingCommand(DriveSubsystem drive) {
    this.drive = drive;
    pigeon = drive.pigeon;
    gyroData = new double[3];

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    pigeon.getYawPitchRoll(gyroData);
  }

  @Override
  public void execute() {
    double forward = 0;
    double strafe = 0;

    if (Math.abs(gyroData[1]) > Constants.ANGLE_TOLERANCE) {
      forward = Constants.FORWARD_VELOCITY;
    }
    if (Math.abs(gyroData[2]) > Constants.ANGLE_TOLERANCE) {
      strafe = Constants.STRAFE_VELOCITY;
    }
    if (gyroData[1] < Constants.ANGLE_TOLERANCE) {
      forward *= -1;
    }
    if (gyroData[2] > Constants.ANGLE_TOLERANCE) {
      strafe *= -1;
    }
    drive.setChassisSpeeds(strafe, forward, 0, true);
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
