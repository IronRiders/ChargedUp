package frc.robot;

import frc.robot.subsystems.ManipulatorSubsystem;
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
  private GrabObject grabRequest = GrabObject.CONE;

  public RobotContainer() {
    configureBindings();
    lights.setColorGrabObject(grabRequest);
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        new RunCommand(
            () ->
                drive.setChassisSpeeds(
                    scaledDeadBand(xboxController.getLeftX(), 1),
                    scaledDeadBand(xboxController.getLeftY(), 1),
                    -scaledDeadBand(xboxController.getRightX(), 1),
                    false),
            drive));

    // Game Piece Tracking
    // controller
    //     .button(34)
    //     .whileTrue(
    //         new PathToPose(
    //             drive, () -> vision.fieldElementTracking(drive.getPose2d(),
    // vision.camera).get()));

    // On The Fly Pathing to Every Station
    // controller
    //     .button(100)
    //     .onTrue(new PathToPose(drive, () ->
    // FieldUtil.getTransformPoseStation(FieldUtil.Station1)));
    // controller
    //     .button(101)
    //     .onTrue(new PathToPose(drive, () ->
    // FieldUtil.getTransformPoseStation(FieldUtil.Station2)));
    // controller
    //     .button(102)
    //     .onTrue(new PathToPose(drive, () ->
    // FieldUtil.getTransformPoseStation(FieldUtil.Station3)));
    // controller
    //     .button(103)
    //     .onTrue(new PathToPose(drive, () ->
    // FieldUtil.getTransformPoseStation(FieldUtil.Station4)));
    // controller
    //     .button(104)
    //     .onTrue(new PathToPose(drive, () ->
    // FieldUtil.getTransformPoseStation(FieldUtil.Station5)));
    // controller
    //     .button(105)
    //     .onTrue(new PathToPose(drive, () ->
    // FieldUtil.getTransformPoseStation(FieldUtil.Station6)));
    // controller
    //     .onTrue(new PathToPose(drive, () ->
    // FieldUtil.getTransformPoseStation(FieldUtil.Station7)));
    // controller
    //     .button(107)
    //     .onTrue(new PathToPose(drive, () ->
    // FieldUtil.getTransformPoseStation(FieldUtil.Station8)));
    // controller
    //     .button(108)
    //     .onTrue(new PathToPose(drive, () ->
    // FieldUtil.getTransformPoseStation(FieldUtil.Station9)));
    controller
        .button(7)
        .onTrue(
            new InstantCommand(
                () -> {
                  if (grabRequest == GrabObject.CONE) {
                    // Switch to Cube
                    grabRequest = GrabObject.BOX;
                  } else {
                    // Switch to Cone
                    grabRequest = GrabObject.CONE;
                  }
                  lights.setColorGrabObject(grabRequest);
                },
                lights));

    controller.button(1).whileTrue(new GrabManipulatorCommand(manipulator, GrabObject.BOX));
    controller.button(2).whileTrue(new ReleaseManipulatorCommand(manipulator));

    controller
        .button(9)
        .onTrue(
            Commands.runOnce(
                () -> {
                  pivot.setGoal(Units.degreesToRadians(Constants.L2ANGLE));
                  pivot.enable();
                },
                pivot));

    controller
        .button(10)
        .onTrue(
            Commands.runOnce(
                () -> {
                  pivot.setGoal(Units.degreesToRadians(Constants.L3ANGLE));
                  pivot.enable();
                },
                pivot));

    controller
        .button(8)
        .onTrue(
            Commands.runOnce(
                () -> {
                  pivot.setGoal(Constants.ARM_OFF_SET_RADS);
                  pivot.enable();
                },
                pivot));
    controller
        .button(11)
        .onTrue(
            Commands.runOnce(
                () -> {
                  pivot.setGoal(Units.degreesToRadians(Constants.LGROUND));
                  pivot.enable();
                },
                pivot));
    // human substation pickup
    controller
        .button(12)
        .onTrue(
            Commands.runOnce(
                () -> {
                  pivot.setGoal(Units.degreesToRadians(Constants.LHUMAN));
                  pivot.enable();
                },
                pivot));

    controller.button(5).whileTrue(new StartEndCommand(arm::extend, arm::stop, arm));
    controller.button(3).whileTrue(new StartEndCommand(arm::retract, arm::stop, arm));

    xboxController.button(2).whileTrue(new PreLevelingCommand(drive));
    xboxController.button(1).whileTrue(new AutoLevelingCommand(drive));
    // Set up shuffleboard
    xboxController.button(3).onTrue(Commands.runOnce(() -> drive.pigeon.reset(), drive));
  }

  public Command getAutonomousCommand() {
    return autoOptions.getAutoCommand();
  }

  public void traj() {
    SmartDashboard.putData("field", drive.field);
  }

  private double scaledDeadBand(double value, double exp) {
    double value1 = MathUtil.applyDeadband(value, Constants.DEADBAND);
    double test = Math.signum(value1) * Math.pow(Math.abs(value1), exp);
    return test;
  }
}
