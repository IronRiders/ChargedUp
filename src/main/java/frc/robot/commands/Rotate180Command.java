package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class Rotate180Command extends CommandBase {
    private final DriveSubsystem driveSubsytem;
    private double initialHeading;
    private boolean clockwise;

    public Rotate180Command(DriveSubsystem driveSubsystem) {
        this(true, driveSubsystem);
    }

    public Rotate180Command(boolean clockwise, DriveSubsystem driveSubsytem) {
        this.clockwise = clockwise;
        this.driveSubsytem = driveSubsytem;
  
        addRequirements(driveSubsytem);
    }

    @Override()
    public void initialize() {
        this.initialHeading = driveSubsytem.pigeon.getYaw();
        SmartDashboard.putNumber("Initial Yaw", driveSubsytem.pigeon.getYaw());
        SmartDashboard.putNumber("Initial Pose", driveSubsytem.getPose2d().getRotation().getDegrees());
    }

    @Override
    public void execute() {
      driveSubsytem.setChassisSpeeds(
          0,
          0,
          clockwise ? -Constants.STRAIGHTEN_ROBOT_TURN_SPEED : Constants.STRAIGHTEN_ROBOT_TURN_SPEED, 
          true);
    }

    public boolean isFinished() {
        double yaw = Math.abs(driveSubsytem.pigeon.getYaw() - initialHeading);
        return 180 - Constants.STRAIGHTEN_TOLERANCE_ANGLE <= yaw && yaw <= 180 + Constants.STRAIGHTEN_TOLERANCE_ANGLE;
    }
}