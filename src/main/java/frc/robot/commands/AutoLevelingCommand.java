package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LightsSubsystem;

public class AutoLevelingCommand extends CommandBase {

  public final DriveSubsystem drive;

  // Robot is balanced at 0 degrees at charge station
  double balanceSetPoint = 0;

  double kP = 0.0056;

  double balanceEffort;

  // Robot aligns to 0 degrees
  double angleSetPoint = 0;

  double kTurn = 0.009;

  double turningEffort;

  public AutoLevelingCommand(DriveSubsystem drive) {
    this.drive = drive;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // turningEffort = (angleSetPoint - (drive.pigeon.getAngle() % 360)) * kTurn;
    balanceEffort = (balanceSetPoint - drive.pigeon.getPitch()) * kP;

    drive.setChassisSpeeds(0, -balanceEffort, 0, true);
    LightsSubsystem.setLightPattern(LightsSubsystem.LightPattern.CHARGING_STATION);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    new Thread(() -> {
      try {Thread.sleep(500);} catch (Exception ignored) {}
      LightsSubsystem.setLightPattern(LightsSubsystem.LightPattern.RAINBOW);
    }).start();
  }

  @Override
  public boolean isFinished() {
    // return Math.abs(Subsystems.driveSubsystem.getGyroPitch()) < 2;
    return false;
  }
}
