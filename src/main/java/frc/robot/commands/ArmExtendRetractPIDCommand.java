package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmExtendRetractPIDCommand extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final PIDController pidController;
  private final double setpoint;
  private double speed;

  public ArmExtendRetractPIDCommand(ArmSubsystem armSubsystem, double setpoint) {
    this.armSubsystem = armSubsystem;
    this.pidController =
        new PIDController(
            Constants.ARM_EXTEND_RETRACT_PID_KP,
            Constants.ARM_EXTEND_RETRACT_PID_KI,
            Constants.ARM_EXTEND_RETRACT_PID_KD);
    this.setpoint = setpoint;
    pidController.setSetpoint(setpoint);

    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    pidController.reset();
    armSubsystem.resetBoxClimberMotorEncoder();
  }

  @Override
  public void execute() {
    speed = pidController.calculate(armSubsystem.getArmMotorEncoderDistance(), setpoint);
    armSubsystem.setBoxClimberMotor(speed);
  }

  @Override
  public boolean isFinished() {
    if (speed <= Constants.ARM_EXTEND_RETRACT_PID_TOLERANCE) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }
}
