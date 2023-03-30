package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.GrabObject;

public class GrabManipulatorCommand extends CommandBase {
  private final ManipulatorSubsystem manipulatorSubsystem;
  private final GrabObject grabObject;

  public GrabManipulatorCommand(ManipulatorSubsystem manipulatorSubsystem, GrabObject grabObject) {
    this.manipulatorSubsystem = manipulatorSubsystem;
    this.grabObject = grabObject;
    addRequirements(manipulatorSubsystem);
  }

  @Override
  public void initialize() {
    manipulatorSubsystem.grab(grabObject);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
