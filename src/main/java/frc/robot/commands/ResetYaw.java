package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MecanumDrive;

public class ResetYaw extends CommandBase{
    public void reset() {
        MecanumDrive.getPigeon().setYaw(0);
    }
}
