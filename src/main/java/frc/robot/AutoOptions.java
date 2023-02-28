package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.MecanumPathFollower;
import frc.robot.subsystems.DriveSubsytem;

public class AutoOptions {

  private SendableChooser<CommandBase> autoOptions = new SendableChooser<>();
  private DriveSubsytem drive;

  public AutoOptions(DriveSubsytem drive) {
    this.drive = drive;
    autoOptions.setDefaultOption(
        "FirstPickUpLeftSide",
        new MecanumPathFollower(
            drive, "FirstPickUpLeftSide", Constants.TooFastAutoConstraints, true));
    autoOptions.addOption(
        "FirstDropOffLeftSide",
        new MecanumPathFollower(
            drive, "FirstDropOffLeftSide", Constants.TooFastAutoConstraints, true));
    autoOptions.addOption(
        "SecondPickUpLeftSide",
        new MecanumPathFollower(
            drive, "SecondPickUpLeftSide", Constants.TooFastAutoConstraints, true));
    autoOptions.addOption(
        "SecondDropOffLeftSide",
        new MecanumPathFollower(
            drive, "SecondDropOffLeftSide", Constants.TooFastAutoConstraints, true));
    autoOptions.addOption(
        "Test Path",
        new MecanumPathFollower(drive, "Test Path", Constants.TooFastAutoConstraints, true));
    autoOptions.addOption("Three Piece", threePieceAuto());

    submit();
  }

  public CommandBase getAutoCommand() {
    var cmd = autoOptions.getSelected();
    if (cmd == null) {
      cmd = Commands.none();
    }
    return cmd;
  }

  public void submit() {
    SmartDashboard.putData("Auto Options", autoOptions);
  }

  public SequentialCommandGroup threePieceAuto() {
    return new SequentialCommandGroup(
      new MecanumPathFollower(drive, "FirstPickUpLeftSide", Constants.MediumAutoConstraints, true),
      new WaitCommand(2),
      new MecanumPathFollower(drive, "FirstDropOffLeftSide", Constants.MediumAutoConstraints, false),
      new WaitCommand(2),
      new MecanumPathFollower(drive, "SecondPickUpLeftSide", Constants.MediumAutoConstraints, false),
      new WaitCommand(2),
      new MecanumPathFollower(drive, "SecondDropOffLeftSide", Constants.MediumAutoConstraints, false));
    }
}
