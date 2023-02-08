package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ResetYawCommand extends CommandBase{
    public void reset() {
        new RobotContainer().drive.pigeon.setYaw(0);
    }
}
