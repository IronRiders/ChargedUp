package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsytem;

public class StraightenRobotCommand extends CommandBase {
    private final DriveSubsytem driveSubsytem;

    public StraightenRobotCommand(DriveSubsytem driveSubsytem) {
        this.driveSubsytem = driveSubsytem;

        addRequirements(driveSubsytem);
    }

    @Override
    public void execute() {
        driveSubsytem.setChassisSpeeds(
            0,
            0,
            (driveSubsytem.pigeon.getRotation2d().getDegrees() % 360 > 180 ? -1 : 1)
                * Constants.STRAIGHTEN_ROBOT_TURN_SPEED,
            false);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(driveSubsytem.pigeon.getRotation2d().getDegrees() % 360)
            < Constants.STRAIGHTEN_TALORANCE_ANGLE);
    }
}
