package frc.robot;

import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.util.FieldUtil;
import frc.robot.util.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.GrabManipulatorCommand;
import frc.robot.commands.PathToPose;
import frc.robot.commands.ReleaseManipulatorCommand;
import frc.robot.subsystems.*;

public class RobotContainer {

  public final ManipulatorSubsystem manipulator = new ManipulatorSubsystem();
  public final Vision vision = new Vision();
  public final DriveSubsystem drive = new DriveSubsystem();
  public final PivotSubsystem pivot = new PivotSubsystem();
  public final ArmSubsystem arm = new ArmSubsystem();
  public final LightsSubsystem lights = new LightsSubsystem();
  private final CommandJoystick controller = new CommandJoystick(0);
  private final CommandXboxController xboxController = new CommandXboxController(1);
  private final AutoOptions autoOptions = new AutoOptions(drive, pivot, arm, manipulator);
  private GrabObject grabRequest = GrabObject.CONE;

  public RobotContainer() {
    configureBindings();
    lights.setColorGrabObject(grabRequest);
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        new RunCommand(
            () -> drive.setChassisSpeeds(
                scaledDeadBand(xboxController.getLeftX(), 1),
                scaledDeadBand(xboxController.getLeftY(), 1),
                -scaledDeadBand(xboxController.getRightX(), 1),
                false),
            drive));

    xboxController.button(1).onTrue(new PathToPose(drive, () -> vision.tagLocalization(Units.inchesToMeters(18), 0, Units.degreesToRadians(180), drive.getPose2d(), vision.limelight).get()));
     controller.button(101).onTrue(new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station2)));
  /*   controller
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
                lights));*/
      controller.button(7).onTrue(autoOptions.L3ConeAutoArmMove());
      controller.button(12).onTrue(autoOptions.L2ConeAutoArmMove());

    controller.button(1).whileTrue(new GrabManipulatorCommand(manipulator, GrabObject.BOX));
    controller.button(2).whileTrue(new ReleaseManipulatorCommand(manipulator));

    controller
        .button(9)
        .onTrue(autoOptions.L2CubeAutoArmMove());

    controller
        .button(10)
        .onTrue(autoOptions.L3CubeAutoArmMove());

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
        .onTrue(autoOptions.L1AutoArmMove());
    // human substation pickup
   /*  controller
        .button(12)
        .onTrue(
            Commands.runOnce(
                () -> {
                  pivot.setGoal(Units.degreesToRadians(Constants.LHUMAN));
                  pivot.enable();
                },
                pivot));*/

    controller.button(5).whileTrue(new StartEndCommand(arm::extend, arm::stop, arm));
    controller.button(3).whileTrue(new StartEndCommand(arm::retract, arm::stop, arm));

    xboxController.button(2).onTrue(autoOptions.loadingStationAutoArmMoveDown());
   // xboxController.button(1).whileTrue(new AutoLevelingCommand(drive));
    xboxController.button(4).onTrue(autoOptions.loadingStationAutoArmMoveUp());
    // Set up shuffleboard
    xboxController.button(3).onTrue(Commands.runOnce(() -> drive.pigeon.reset(), drive));
    xboxController.button(5).onTrue(autoOptions.GroundPickUp());
    xboxController.button(6).onTrue(autoOptions.GroundDropOff());
  }

  public Command getAutonomousCommand() {
    return autoOptions.PlaceAndbalance();
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
