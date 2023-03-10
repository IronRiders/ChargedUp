package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.UltrasonicSensorSubsystem;
import frc.robot.subsystems.GrabObject;

public class UltrasonicSensorCommand extends CommandBase {
  private final UltrasonicSensorSubsystem ultrasonicSensorSubsystem;
  private final ManipulatorSubsystem manipulatorSubsystem;
  private final Enum coneGrabber;

  public UltrasonicSensorCommand(
      UltrasonicSensorSubsystem ultrasonicSensorSubsystem,
      ManipulatorSubsystem manipulatorSubsystem,
      GrabObject coneGrabber) {
    this.ultrasonicSensorSubsystem = ultrasonicSensorSubsystem;
    this.manipulatorSubsystem = manipulatorSubsystem;
    this.coneGrabber = coneGrabber;
    addRequirements(manipulatorSubsystem);
  }

  @Override
  public void initialize() {
    if (ultrasonicSensorSubsystem.isObjectNearby()) {
      if (coneGrabber == GrabObject.CONE) {
        manipulatorSubsystem.grab(GrabObject.CONE);
      } else {
        manipulatorSubsystem.grab(GrabObject.BOX);
      }
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
