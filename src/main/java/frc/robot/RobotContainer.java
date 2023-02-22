package frc.robot;

import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.GrabObject;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.ArmExtendRetractPIDCommand;
import frc.robot.commands.ArmRaiseLowerPIDCommand;
import frc.robot.commands.AutoLevelingCommand;
import frc.robot.commands.CurrentLimitsManipulatorCommand;
import frc.robot.commands.GrabManipulatorCommand;
import frc.robot.commands.ManipulatorPIDCommand;
import frc.robot.commands.ReleaseManipulatorCommand;
import frc.robot.subsystems.*;

public class RobotContainer {

  public final ManipulatorSubsystem manipulator = new ManipulatorSubsystem();
  // one of the two following lines of code must be commented out at all times
  // public final DifferentialDrive drive = new DifferentialDrive();
  public final DriveSubsytem drive = new DriveSubsytem();
  public final ArmSubsystem arm = new ArmSubsystem();
  private final CommandJoystick controller = new CommandJoystick(0);

  public RobotContainer() {

    drive.setDefaultCommand(
        new RunCommand(
            () ->
                drive.setChassisSpeeds(
                    joystickResponse(controller.getRawAxis(0)),
                    joystickResponse(controller.getRawAxis(1)),
                    joystickResponse(controller.getRawAxis(2)),
                    false),
            drive));

    configureBindings();
  }

  private void configureBindings() {
    controller.button(19).whileTrue(Commands.startEnd(() -> arm.extend(), () -> arm.stop(), arm));
    controller.button(20).whileTrue(Commands.startEnd(() -> arm.retract(), () -> arm.stop(), arm));
    controller.button(21).whileTrue(Commands.startEnd(() -> arm.raise(), () -> arm.stop(), arm));
    controller.button(22).whileTrue(Commands.startEnd(() -> arm.lower(), () -> arm.stop(), arm));
    controller
        .button(1)
        .onTrue(new ArmRaiseLowerPIDCommand(arm, Constants.ARM_RAISE_LOWER_SETPOINT));
    controller
        .button(2)
        .onTrue(new ArmExtendRetractPIDCommand(arm, Constants.ARM_EXTEND_RETRACT_SETPOINT));
    controller
        .button(3)
        .onTrue(new ManipulatorPIDCommand(manipulator, Constants.MANIPULATOR_SETPOINT));
    controller.button(4).whileTrue(new AutoLevelingCommand(drive));
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
    controller
        .button(34)
        .onTrue(
            new CurrentLimitsManipulatorCommand(
                manipulator, GrabObject.CONE, Constants.MANIPULATOR_SETPOINT));
    controller
        .button(35)
        .onTrue(
            new CurrentLimitsManipulatorCommand(
                manipulator, GrabObject.BOX, Constants.MANIPULATOR_SETPOINT));
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
