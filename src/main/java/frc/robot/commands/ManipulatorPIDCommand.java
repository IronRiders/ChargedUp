package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ManipulatorPIDCommand extends CommandBase {
  private final ManipulatorSubsystem manipulatorSubsystem;
  private final PIDController pidController;
  private final double setpoint;

  public ManipulatorPIDCommand(ManipulatorSubsystem manipulatorSubsystem, double setpoint) {
    this.manipulatorSubsystem = manipulatorSubsystem;
    this.pidController =
        new PIDController(
            Constants.MANIPULATOR_PID_KP,
            Constants.MANIPULATOR_PID_KI,
            Constants.MANIPULATOR_PID_KD);
    this.setpoint = setpoint;
    pidController.setSetpoint(setpoint);

    addRequirements(manipulatorSubsystem);
  }

  @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {
    double speed =
        pidController.calculate(
            manipulatorSubsystem.getManipulatorMotor1EncoderDistance(), setpoint);
    manipulatorSubsystem.setManipulatorMotors(speed);

    if (speed == 0) {
      manipulatorSubsystem.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    manipulatorSubsystem.stop();
  }
}
