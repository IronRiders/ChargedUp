package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabObject;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ExpelManipulatorCommand extends CommandBase {
  private final ManipulatorSubsystem manipulatorSubsystem;
  boolean grabCone;
  GrabObject grabObject;

  public ExpelManipulatorCommand(ManipulatorSubsystem manipulatorSubsystem, GrabObject grabObject) {
    this.manipulatorSubsystem = manipulatorSubsystem;
    this.grabObject = grabObject;

    addRequirements(manipulatorSubsystem);
  }

  @Override
  public void initialize() {
    manipulatorSubsystem.expel(grabObject);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    manipulatorSubsystem.expel(grabObject);
  }
}
