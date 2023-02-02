package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem.GrabObject;

public class GrabManipulatorCommand extends CommandBase {
  private final ManipulatorSubsystem manipulatorSubsystem;
  private final boolean coneGraber;

  public GrabManipulatorCommand(ManipulatorSubsystem manipulatorSubsystem, boolean coneGraber) {
    this.manipulatorSubsystem = manipulatorSubsystem;
    this.coneGraber = coneGraber;
    addRequirements(manipulatorSubsystem);
  }

  @Override
  public void initialize() {
    if (coneGraber) {
      manipulatorSubsystem.grab(GrabObject.CONE);
    } else {
      manipulatorSubsystem.grab(GrabObject.BOX);
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
