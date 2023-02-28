package frc.robot;

import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.GrabObject;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.ArmExtendRetractPIDCommand;
import frc.robot.commands.ArmRaiseLowerPIDCommand;
import frc.robot.commands.AutoLevelingCommand;
import frc.robot.commands.GrabManipulatorCommand;
import frc.robot.commands.ManipulatorPIDCommand;
import frc.robot.commands.ReleaseManipulatorCommand;
import frc.robot.commands.UltrasonicSensorCommand;
import frc.robot.commands.TagFollowing;
import frc.robot.subsystems.*;

public class RobotContainer {

  public final ManipulatorSubsystem manipulator = new ManipulatorSubsystem();
  // one of the two following lines of code must be commented out at all times
  // public final DifferentialDrive drive = new DifferentialDrive();
  public final DriveSubsytem drive = new DriveSubsytem();
  public final ArmSubsystem arm = new ArmSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();
  private final CommandJoystick controller = new CommandJoystick(0);
  private final UltrasonicSensorSubsystem ultrasonicSensor = new UltrasonicSensorSubsystem();
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
    controller
        .button(29)
        .whileTrue(
            new UltrasonicSensorCommand(ultrasonicSensor, manipulator, GrabObject.BOX));
    controller
        .button(30)
        .onTrue(
            new UltrasonicSensorCommand(ultrasonicSensor, manipulator, GrabObject.CONE));
        .button(50)
        .onTrue(
            new TagFollowing(
                drive,
                () -> {
                  var target = vision.camera.getLatestResult().getBestTarget();
                  if (target == null) return null;
                  return new Pose3d(drive.getPose2d())
                      .plus(Constants.RobotToCam)
                      .plus(target.getBestCameraToTarget())
                      .toPose2d()
                      .plus(
                          new Transform2d(
                              new Translation2d(Units.inchesToMeters(36 + 30 / 2.0), 0),
                              new Rotation2d(Math.PI)));
                }));

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
