// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.GrabManipulatorCommand;
import frc.robot.commands.ReleaseManipulatorCommand;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final ManipulatorSubsystem manipulator = new ManipulatorSubsystem();

  // one of the two following lines of code must be commented out at all times
  public final DiferentialDrive drive = new DiferentialDrive();
  // public final MecanumDrive drive = new MecanumDrive();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick controller = new CommandJoystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Drive
    drive.setDefaultCommand(
        new RunCommand(
            () ->
                drive.updateSpeed(
                    joystickResponse(controller.getRawAxis(0)),
                    joystickResponse(controller.getRawAxis(1)),
                    joystickResponse(controller.getRawAxis(3)),
                    true),
            drive));

    // Configure the trigger bindings
    configureBindings();
  }

  // Bind triggers to Commands
  private void configureBindings() {
    controller.button(9).onTrue(new GrabManipulatorCommand(manipulator));
    controller.button(5).onTrue(new ReleaseManipulatorCommand(manipulator));
  }

  // public Command getAutonomousCommand() {
  // }

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
