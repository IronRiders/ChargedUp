package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class StraightenRobot extends CommandBase{
    public void straighten() {
        try (PIDController pid = new PIDController(Constants.STRAIGHTEN_ROBOT_P, Constants.STRAIGHTEN_ROBOT_I, Constants.STRAIGHTEN_ROBOT_D)) {
            pid.setTolerance(Constants.STRAIGHTEN_TALORNCE_ANGLE);
            pid.setSetpoint(0);
            pid.enableContinuousInput(-180, 180);
        }
    }
}
