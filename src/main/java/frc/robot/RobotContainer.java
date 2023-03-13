package frc.robot;

import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.util.FieldUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoLevelingCommand;
import frc.robot.commands.GrabManipulatorCommand;
import frc.robot.commands.ReleaseManipulatorCommand;
import frc.robot.commands.BurnFlashCommand;
import frc.robot.commands.PathToPose;
import frc.robot.commands.PreLevelingCommand;
import frc.robot.subsystems.*;

public class RobotContainer {

  public final ManipulatorSubsystem manipulator = new ManipulatorSubsystem();
  public final DriveSubsystem drive = new DriveSubsystem();
  private final Vision vision = new Vision();
  public final PivotSubsystem pivot = new PivotSubsystem();
  public final ArmSubsystem arm = new ArmSubsystem();
  public final LightsSubsystem lights = new LightsSubsystem();
  private final CommandJoystick controller = new CommandJoystick(0);
  private final CommandXboxController xboxController = new CommandXboxController(1);
  private final AutoOptions autoOptions = new AutoOptions(drive);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        new RunCommand(
            () ->
                drive.setChassisSpeeds(
                    ScaledDeadBand(xboxController.getLeftX(), 1),
                    ScaledDeadBand(xboxController.getLeftY(), 1),
                    -ScaledDeadBand(xboxController.getRightX(), 1),
                    false),
            drive));

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
        .button(39)
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
    controller.button(54).whileTrue(new AutoLevelingCommand(drive));
    controller.button(31).whileTrue(new GrabManipulatorCommand(manipulator, GrabObject.CONE));
    controller.button(32).whileTrue(new GrabManipulatorCommand(manipulator, GrabObject.BOX));
    controller.button(33).whileTrue(new ReleaseManipulatorCommand(manipulator));

    controller
        .button(3)
        .onTrue(
            Commands.runOnce(
                () -> {
                  pivot.setGoal(Units.degreesToRadians(Constants.L2ANGLE));
                  pivot.enable();
                },
                pivot));

    controller
        .button(4)
        .onTrue(
            Commands.runOnce(
                () -> {
                  pivot.setGoal(Units.degreesToRadians(110));
                  pivot.enable();
                },
                pivot));

    controller
        .button(5)
        .onTrue(
            Commands.runOnce(
                () -> {
                  pivot.setGoal(Constants.ARM_OFF_SET_RADS);
                  pivot.enable();
                },
                pivot));

    controller.button(11).whileTrue(new StartEndCommand(arm::extend, arm::stop, arm));
    controller.button(12).whileTrue(new StartEndCommand(arm::retract, arm::stop, arm));

    xboxController.button(2).whileTrue(new PreLevelingCommand(drive));
    xboxController.button(1).whileTrue(new AutoLevelingCommand(drive));

    controller.button(10).whileTrue(new GrabManipulatorCommand(manipulator, GrabObject.CONE));
    controller.button(11).whileTrue(new GrabManipulatorCommand(manipulator, GrabObject.BOX));
    controller.button(12).whileTrue(new ReleaseManipulatorCommand(manipulator));

    // Set up shuffleboard
    SmartDashboard.putData("Reset Gyro", Commands.runOnce(() -> drive.pigeon.reset(), drive));
    SmartDashboard.putData("Burn Flash", new BurnFlashCommand(drive, pivot, arm, manipulator));
    SmartDashboard.putData("Reset Gyro", Commands.runOnce(() -> drive.pigeon.reset(), drive));
  }

  public Command getAutonomousCommand() {
    return autoOptions.getAutoCommand();
  }

  public void traj() {
    SmartDashboard.putData("field", drive.field);
  }

  private double ScaledDeadBand(double value, double exp) {
    double value1 = MathUtil.applyDeadband(value, Constants.DEADBAND);
    double test = Math.signum(value1) * Math.pow(Math.abs(value1), exp);
    return test;
  }
}
