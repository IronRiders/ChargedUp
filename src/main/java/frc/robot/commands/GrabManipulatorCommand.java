package frc.robot.commands;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.GrabObject;

public class GrabManipulatorCommand extends CommandBase {
  private final ManipulatorSubsystem manipulatorSubsystem;
  private final GrabObject coneGraber;

  public GrabManipulatorCommand(ManipulatorSubsystem manipulatorSubsystem, GrabObject coneGraber) {
    this.manipulatorSubsystem = manipulatorSubsystem;
    this.coneGraber = coneGraber;
    addRequirements(manipulatorSubsystem);
    
  }
  PowerDistribution pdh = new PowerDistribution();

  @Override
  public void initialize() {
    manipulatorSubsystem.grab(coneGraber);
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
