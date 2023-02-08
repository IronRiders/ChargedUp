package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmRaiseLowerPIDCommand extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final PIDController pidController;
  private final double setpoint;

  public ArmRaiseLowerPIDCommand(ArmSubsystem armSubsystem, double setpoint) {
    this.armSubsystem = armSubsystem;
    this.pidController = new PIDController(Constants.ARM_RAISE_LOWER_PID_KP, Constants.ARM_RAISE_LOWER_PID_KI, Constants.ARM_RAISE_LOWER_PID_KD);
    this.setpoint = setpoint;
    pidController.setSetpoint(setpoint);

    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {
    double speed = pidController.calculate(armSubsystem.getBoxClimberEncoderDistance(), setpoint);
    armSubsystem.setArmMotor(speed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }
}
