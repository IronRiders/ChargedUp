package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLevelingCommand extends CommandBase {

  public final DriveSubsystem drive;

  // Robot is balanced at 0 degrees at charge station
  double balanceSetpoint = 0;

  double kP = 0.0056;

  double balanceEffort;

  // Robot aligns to 0 degrees
  double angleSetpoint = 0;

  double kTurn = 0.009;

  double turningEffort;

  public AutoLevelingCommand(DriveSubsystem drive) {
    this.drive = drive;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // turningEffort = (angleSetpoint - (drive.pigeon.getAngle() % 360)) * kTurn;
    balanceEffort = (balanceSetpoint - drive.pigeon.getPitch()) * kP;

    drive.setChassisSpeeds(0, -balanceEffort, 0, true);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    // return Math.abs(Subsystems.driveSubsystem.getGyroPitch()) < 2;
    return false;
  }
}
