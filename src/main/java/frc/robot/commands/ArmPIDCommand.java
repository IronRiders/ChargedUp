package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPIDCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final PIDController pidController;
    private final double setpoint;

    public ArmPIDCommand(ArmSubsystem armSubsystem, double setpoint) {
        this.armSubsystem = armSubsystem;
        this.pidController = new PIDController(0, 0, 0);
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
        double speed = pidController.calculate(armSubsystem.getEncoderDistance(), setpoint);
        armSubsystem.setBoxClimberMotor(speed);
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
