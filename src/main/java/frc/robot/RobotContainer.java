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
import frc.robot.commands.AutoLevelingCommand;
import frc.robot.commands.GrabManipulatorCommand;
import frc.robot.commands.ReleaseManipulatorCommand;
import frc.robot.commands.PathToPose;
import frc.robot.subsystems.*;

public class RobotContainer {

  public final ManipulatorSubsystem manipulator = new ManipulatorSubsystem();
  public final DriveSubsytem drive = new DriveSubsytem();
  public final ExtendingArmSubsystem armExtend = new ExtendingArmSubsystem();
  public final RaiseLowerArmSubsystem armRaise = new RaiseLowerArmSubsystem();
  public final LightsSubsystem lights = new LightsSubsystem();
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
    controller
        .button(34)
        .whileTrue(
            new PathToPose(drive, () -> vision.fieldElementTracking(drive.getPose2d()).get()));

    // On The Fly Pathing to Every Station
    controller
        .button(100)
        .onTrue(new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station1)));
    controller
        .button(101)
        .onTrue(new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station2)));
    controller
        .button(102)
        .onTrue(new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station3)));
    controller
        .button(103)
        .onTrue(new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station4)));
    controller
        .button(104)
        .onTrue(new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station5)));
    controller
        .button(105)
        .onTrue(new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station6)));
    controller
        .button(106)
        .onTrue(new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station7)));
    controller
        .button(107)
        .onTrue(new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station8)));
    controller
        .button(108)
        .onTrue(new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station9)));

    // Switching Pipelines manually
    controller
        .button(9)
        .onTrue(
            new InstantCommand(
                () -> {
                  if (vision.camera.getPipelineIndex() == 4) {
                    vision.camera.setPipelineIndex(2);
                    lights.setColorHSV(253, 224, 25);
                    return;
                  }
                  vision.camera.setPipelineIndex(4);
                  lights.setColorHSV(259, 100, 70);
                }));

    controller
        .button(3)
        .whileTrue(Commands.startEnd(() -> armExtend.extend(), () -> armExtend.stop(), armExtend));
    controller
        .button(5)
        .whileTrue(Commands.startEnd(() -> armExtend.retract(), () -> armExtend.stop(), armExtend));
    controller
        .button(4)
        .whileTrue(Commands.startEnd(() -> armRaise.raise(), () -> armRaise.stop(), armRaise));
    controller
        .button(6)
        .whileTrue(Commands.startEnd(() -> armRaise.lower(), () -> armRaise.stop(), armRaise));
    /* controller
        .button(1)
        .onTrue(new ArmRaiseLowerPIDCommand(armRaise, Constants.ARM_RAISE_LOWER_SETPOINT));
    controller
        .button(2)
        .onTrue(new ArmExtendRetractPIDCommand(armExtend, Constants.ARM_EXTEND_RETRACT_SETPOINT));
    controller
        .button(3)
        .onTrue(new ManipulatorPIDCommand(manipulator, Constants.MANIPULATOR_SETPOINT));
    */
    controller.button(1).whileTrue(new AutoLevelingCommand(drive));
    controller
        .button(10)
        .onTrue(
            new GrabManipulatorCommand(manipulator, GrabObject.CONE)); // Button For Grabbing Cones
    controller
        .button(11)
        .onTrue(
            new GrabManipulatorCommand(manipulator, GrabObject.BOX)); // Button For Grabbing Boxes
    controller
        .button(12)
        .onTrue(new ReleaseManipulatorCommand(manipulator)); // Button For Releasing

    SmartDashboard.putData("Reset Gyro", Commands.runOnce(() -> drive.pigeon.reset(), drive));
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
