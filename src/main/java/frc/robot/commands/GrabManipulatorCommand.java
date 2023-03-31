package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.GrabObject;

public class GrabManipulatorCommand extends CommandBase {
  private final ManipulatorSubsystem manipulatorSubsystem;
  private final GrabObject grabObject;

  public GrabManipulatorCommand(ManipulatorSubsystem manipulatorSubsystem) {
    this.manipulatorSubsystem = manipulatorSubsystem;
    this.grabObject = RobotContainer.grabRequest;
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

  @Override
  public void end(boolean interrupted) {
    manipulatorSubsystem.stop();
  }
}
