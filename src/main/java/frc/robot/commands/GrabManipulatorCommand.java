package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.GrabObject;

public class GrabManipulatorCommand extends CommandBase {
  private final ManipulatorSubsystem manipulatorSubsystem;
  private final Enum coneGrabber;
  private final GrabObject coneGraber;

  public GrabManipulatorCommand(ManipulatorSubsystem manipulatorSubsystem, GrabObject coneGraber) {
    this.manipulatorSubsystem = manipulatorSubsystem;
    this.coneGrabber = coneGraber;
    addRequirements(manipulatorSubsystem);
  }

  @Override
  public void initialize() {
    if (coneGrabber == GrabObject.CONE) {
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
