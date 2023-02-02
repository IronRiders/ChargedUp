package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem.GrabObject;

public class ReleaseManipulatorCommand extends CommandBase {
  private final ManipulatorSubsystem manipulatorSubsystem;
  boolean grabcone;

  public ReleaseManipulatorCommand(ManipulatorSubsystem manipulatorSubsystem, boolean grabcone) {
    this.manipulatorSubsystem = manipulatorSubsystem;
    this.grabcone = grabcone;
    addRequirements(manipulatorSubsystem);
  }

  @Override
  public void initialize() {
    if (grabcone) {
      manipulatorSubsystem.release(GrabObject.CONE);
    } else {
      manipulatorSubsystem.release(GrabObject.BOX);
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
