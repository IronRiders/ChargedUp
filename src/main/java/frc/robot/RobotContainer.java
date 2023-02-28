package frc.robot;

import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.util.FieldUtil;
import frc.robot.subsystems.GrabObject;

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
  private final Vision vision = new Vision();
  private final CommandJoystick controller = new CommandJoystick(0);
  private final AutoOptions autoOptions = new AutoOptions(drive);

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

    // Game Piece Tracking
    controller.button(34).whileTrue(new PathToPose(drive, () -> vision.fieldElementTracking(drive.getPose2d()).get()));

    // On The Fly Pathing to Every Station
    controller.button(100).onTrue(new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station1)));
    controller.button(101).onTrue(new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station2)));
    controller.button(102).onTrue(new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station3)));
    controller.button(103).onTrue(new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station4)));
    controller.button(104).onTrue(new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station5)));
    controller.button(105).onTrue(new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station6)));
    controller.button(106).onTrue(new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station7)));
    controller.button(107).onTrue(new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station8)));
    controller.button(108).onTrue(new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station9)));


    // Switching Pipelines manually
    controller.button(3).onTrue(new InstantCommand(() -> vision.camera.setPipelineIndex(2)));
    controller.button(4).onTrue(new InstantCommand(() -> vision.camera.setPipelineIndex(4)));

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
