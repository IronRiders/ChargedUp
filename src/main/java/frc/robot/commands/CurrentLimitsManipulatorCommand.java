package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.GrabObject;
import frc.robot.subsystems.ManipulatorSubsystem;

public class CurrentLimitsManipulatorCommand extends CommandBase{
    private final ManipulatorSubsystem manipulator;
    private final GrabObject object;
    private final PIDController pidController;
    private final double setpoint;
    private double speed = 0;

    public CurrentLimitsManipulatorCommand(ManipulatorSubsystem manipulator, GrabObject object, double setpoint){
        this.manipulator = manipulator;
        this.object = object;
        this.pidController =
        new PIDController(
            Constants.MANIPULATOR_PID_KP,
            Constants.MANIPULATOR_PID_KI,
            Constants.MANIPULATOR_PID_KD);
        this.setpoint = setpoint;
        pidController.setSetpoint(setpoint);

        addRequirements(manipulator);
    }

    @Override
    public void initialize() {
        pidController.reset();
        manipulator.resetManipulatorMotor1EncoderDistance();
    }

    @Override
    public void execute() {
        speed =
            pidController.calculate(
              manipulator.getManipulatorMotor1EncoderDistance(), setpoint);
        manipulator.setManipulatorMotors(speed, object);
     }

    @Override
    public boolean isFinished() {
      if (speed <= Constants.MANIPULATOR_PID_TOLERANCE) {
       return true;
     } else {
       return false;
      }
     }

    @Override
    public void end(boolean interrupted) {
      manipulator.stop();
    }
}
