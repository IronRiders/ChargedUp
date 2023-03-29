// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.ManipulatorSubsystem;

// public class ReleaseManipulatorCommand extends CommandBase { // This system could probably be deleted because we can just run the other Grab command and that will act as a release. 
//   private final ManipulatorSubsystem manipulatorSubsystem;   // We could also find a way to check that last grab command that was run and do the reverse of that to release, but I don't think we would need to that 
//   boolean grabcone;

//   public ReleaseManipulatorCommand(ManipulatorSubsystem manipulatorSubsystem) {
//     this.manipulatorSubsystem = manipulatorSubsystem;

//     addRequirements(manipulatorSubsystem);
//   }

//   @Override
//   public void initialize() {
//     manipulatorSubsystem.release();
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }

//   @Override
//   public void end(boolean interrupted) {
//     manipulatorSubsystem.stop();
//   }
// }
