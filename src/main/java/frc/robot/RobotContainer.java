package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.GrabManipulatorCommand;
import frc.robot.commands.ReleaseManipulatorCommand;
import frc.robot.subsystems.*;

public class RobotContainer {
  
  public final ManipulatorSubsystem manipulator = new ManipulatorSubsystem();
  public final DriveSubsytem drive = new DriveSubsytem();

  private final CommandJoystick controller = new CommandJoystick(0);

  public RobotContainer() {

    drive.setDefaultCommand(
        new RunCommand(
            () ->
                drive.setChassisSpeeds(
                    joystickResponse(controller.getRawAxis(0) * 0.2),
                    joystickResponse(controller.getRawAxis(1) * 0.2),
                    joystickResponse(controller.getRawAxis(2) * 0.2),
                    false),
            drive));

    configureBindings();
  }

  private void configureBindings() {
    controller.button(9).onTrue(new GrabManipulatorCommand(manipulator));
    controller.button(5).onTrue(new ReleaseManipulatorCommand(manipulator));
  }

   public Command getAutonomousCommand() {
    return null;
   }

  private double joystickResponse(double raw) {
    double deadband = SmartDashboard.getNumber("deadband", Constants.DEADBAND);
    double deadbanded = 0.0;
    if (raw > deadband) {
      deadbanded = (raw - deadband) / (1 - deadband);
    } else if (raw < -deadband) {
      deadbanded = (raw + deadband) / (1 - deadband);
    }
    double exponent = SmartDashboard.getNumber("exponent", Constants.EXPONENT) + 1;
    return Math.pow(Math.abs(deadbanded), exponent) * Math.signum(deadbanded);
  }
}
