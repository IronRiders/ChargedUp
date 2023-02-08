package frc.robot;

import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.GrabObject;
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
  // one of the two following lines of code must be commented out at all times
  // public final DifferentialDrive drive = new DifferentialDrive();
  public final MecanumDrive drive = new MecanumDrive();

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
    controller
        .button(31)
        .onTrue(
            new GrabManipulatorCommand(manipulator, GrabObject.CONE)); // Button For Grabbing Cones
    controller
        .button(32)
        .onTrue(
            new GrabManipulatorCommand(manipulator, GrabObject.BOX)); // Button For Grabbing Boxes
    controller
        .button(33)
        .onTrue(new ReleaseManipulatorCommand(manipulator)); // Button For Releasing
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
