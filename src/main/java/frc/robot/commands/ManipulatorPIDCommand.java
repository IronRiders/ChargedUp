// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.subsystems.ManipulatorSubsystem;

// public class ManipulatorPIDCommand extends CommandBase {
//   private final ManipulatorSubsystem manipulator;
//   private final PIDController pidController;
//   private final double setpoint;
//   private double speed;

//   public ManipulatorPIDCommand(ManipulatorSubsystem manipulator, double setpoint) {
//     this.manipulator = manipulator;
//     this.pidController =
//         new PIDController(
//             Constants.MANIPULATOR_PID_KP,
//             Constants.MANIPULATOR_PID_KI,
//             Constants.MANIPULATOR_PID_KD);
//     this.setpoint = setpoint;
//     pidController.setSetpoint(setpoint);

//     addRequirements(manipulator);
//   }

//   @Override
//   public void initialize() {
//     pidController.reset();
//     manipulator.resetManipulatorMotor1EncoderDistance();
//   }

//   @Override
//   public void execute() {
//     speed = pidController.calculate(manipulator.getManipulatorMotor1EncoderDistance(), setpoint);
//     manipulator.setManipulatorMotors(speed);
//   }

//   @Override
//   public boolean isFinished() {
//     if (speed <= Constants.MANIPULATOR_PID_TOLERANCE) {
//       return true;
//     } else {
//       return false;
//     }
//   }

//   @Override
//   public void end(boolean interrupted) {
//     manipulator.stop();
//   }
// }
