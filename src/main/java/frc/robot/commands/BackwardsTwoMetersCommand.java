package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class BackwardsTwoMetersCommand extends CommandBase {
    private final DriveSubsystem driveSubsytem;
    private double initialX;

    public BackwardsTwoMetersCommand(DriveSubsystem driveSubsytem) {
        this.driveSubsytem = driveSubsytem;
  
        addRequirements(driveSubsytem);
    }

    @Override()
    public void initialize() {
        initialX = driveSubsytem.getPose2d().getX();
        SmartDashboard.putNumber("Initial X", driveSubsytem.getPose2d().getX());
    }

    @Override
    public void execute() {
      driveSubsytem.setChassisSpeeds(
          0.0,
          0.1,
          0.,
          false);
    }

    public boolean isFinished() {
        double positionDelta = Math.abs(driveSubsytem.getPose2d().getX() - initialX);
        return positionDelta >= 2;
    }
}