package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.ArmExtendRetractPIDCommand;
import frc.robot.commands.ArmRaiseLowerPIDCommand;
import frc.robot.commands.AutoLevelingCommand;
import frc.robot.commands.GrabManipulatorCommand;
import frc.robot.commands.ManipulatorPIDCommand;
import frc.robot.commands.ReleaseManipulatorCommand;
import frc.robot.commands.PathToPose;
import frc.robot.subsystems.*;

public class RobotContainer {

  public final ManipulatorSubsystem manipulator = new ManipulatorSubsystem();
  public final DriveSubsytem drive = new DriveSubsytem();
  public final ArmSubsystem arm = new ArmSubsystem();
  public final LightsSubsystem lights = new LightsSubsystem();
  private final Vision vision = new Vision();
  private final CommandJoystick controller = new CommandJoystick(0);
  private final AutoOptions autoOptions = new AutoOptions(drive);

  public RobotContainer() {

    drive.setDefaultCommand(
        new RunCommand(
            () -> drive.setChassisSpeeds(
                joystickResponse(controller.getRawAxis(0)),
                joystickResponse(controller.getRawAxis(1)),
                joystickResponse(controller.getRawAxis(2)),
                false),
            drive));

    configureBindings();
  }

  private void configureBindings() {

    // April Tag Tracking
    controller
        .button(13)
        .onTrue(
            new PathToPose(
                drive,
                () -> vision.tagLocalization(30 + 36 / 2, 0.0, Math.PI, drive.getPose2d()).get()));
    // Game Piece Tracking
    controller
        .button(34)
        .whileTrue(
            new PathToPose(drive, () -> vision.fieldElementTracking(drive.getPose2d()).get()));

    controller.button(3).onTrue(new InstantCommand(() -> vision.camera.setPipelineIndex(isCube(true))));
    controller.button(4).onTrue(new InstantCommand(() -> vision.camera.setPipelineIndex(isCube(false))));

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
  }

  public int isCube(boolean isCube) {
    if (isCube) {
      lights.setColorHSV(253, 224, 25);
      return 4;
    }
    lights.setColorHSV(259, 100, 70);
    return 2;
  }

  public Command getAutonomousCommand() {
    return autoOptions.getAutoCommand();
  }

  public void traj() {
    SmartDashboard.putData("field", drive.field);
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
