package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.MecanumPathFollower;
import frc.robot.subsystems.DriveSubsytem;

public class AutoOptions {

  private SendableChooser<CommandBase> autoOptions = new SendableChooser<>();

  public AutoOptions(DriveSubsytem drive) {
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
}
