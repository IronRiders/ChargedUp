package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ResetYaw extends CommandBase{
    public void reset() {
        RobotContainer.pigeon.setYaw(0);
    }
}
