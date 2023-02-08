package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

<<<<<<< HEAD:src/main/java/frc/robot/commands/ArmExtendRetractPIDCommand.java
public class ArmExtendRetractPIDCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final PIDController pidController;
    private final double setpoint;

    public ArmExtendRetractPIDCommand(ArmSubsystem armSubsystem, double setpoint) {
        this.armSubsystem = armSubsystem;
        this.pidController = new PIDController(0, 0, 0);
        this.setpoint = setpoint;
        pidController.setSetpoint(setpoint);
=======
public class BoxClimberPIDCommand extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final PIDController pidController;
  private final double setpoint;

  public BoxClimberPIDCommand(ArmSubsystem armSubsystem, double setpoint) {
    this.armSubsystem = armSubsystem;
    this.pidController = new PIDController(0, 0, 0);
    this.setpoint = setpoint;
    pidController.setSetpoint(setpoint);
>>>>>>> 21278f4d9e3deac1e5ebf6f379ba12165e397a1e:src/main/java/frc/robot/commands/BoxClimberPIDCommand.java

    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    pidController.reset();
  }

<<<<<<< HEAD:src/main/java/frc/robot/commands/ArmExtendRetractPIDCommand.java
    @Override
    public void execute() {
        double speed = pidController.calculate(armSubsystem.getArmMotorEncoderDistance(), setpoint);
        armSubsystem.setBoxClimberMotor(speed);
    }
=======
  @Override
  public void execute() {
    double speed = pidController.calculate(armSubsystem.getEncoderDistance(), setpoint);
    armSubsystem.setBoxClimberMotor(speed);
  }
>>>>>>> 21278f4d9e3deac1e5ebf6f379ba12165e397a1e:src/main/java/frc/robot/commands/BoxClimberPIDCommand.java

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }
}
