package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ReleaseManipulatorCommand extends CommandBase {
  private final ManipulatorSubsystem manipulatorSubsystem;
<<<<<<< HEAD
  // boolean grabcone;
=======
  boolean grabcone;
>>>>>>> f678d10400539094a5551ba047a3e8fea2554ce3

  public ReleaseManipulatorCommand(ManipulatorSubsystem manipulatorSubsystem) {
    this.manipulatorSubsystem = manipulatorSubsystem;

    addRequirements(manipulatorSubsystem);
  }

  @Override
  public void initialize() {
    manipulatorSubsystem.release();
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
