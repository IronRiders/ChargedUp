package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.MecanumPathFollower;
import frc.robot.commands.PathToPose;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.FieldUtil;

public class AutoOptions {

  private SendableChooser<CommandBase> autoOptions = new SendableChooser<>();
  private DriveSubsystem drive;

  public AutoOptions(DriveSubsystem drive) {
    this.drive = drive;

    // Tuning
    autoOptions.setDefaultOption(
        "FirstPickUpLeftSide",
        new MecanumPathFollower(
            drive, "FirstPickUpLeftSide", Constants.MediumAutoConstraints, true));
    autoOptions.addOption(
        "FirstDropOffLeftSide",
        new MecanumPathFollower(
            drive, "FirstDropOffLeftSide", Constants.MediumAutoConstraints, true));
    autoOptions.addOption(
        "SecondPickUpLeftSide",
        new MecanumPathFollower(
            drive, "SecondPickUpLeftSide", Constants.MediumAutoConstraints, true));
    autoOptions.addOption(
        "SecondDropOffLeftSide",
        new MecanumPathFollower(
            drive, "SecondDropOffLeftSide", Constants.MediumAutoConstraints, true));
    autoOptions.addOption(
        "Straight 180 degree turning path",
        new MecanumPathFollower(
            drive, "Straight 180 degree turning path", Constants.SlowAutoConstraints, true));
    autoOptions.addOption(
        "45 degree turning path",
        new MecanumPathFollower(
            drive, "45 degree turning path", Constants.MediumAutoConstraints, true));
    autoOptions.addOption(
        "Charge Right",
        new MecanumPathFollower(drive, "ChargeRight", Constants.MediumAutoConstraints, true));
    autoOptions.addOption(
        "ON The Fly Path Test",
        new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station1)));
    autoOptions.addOption("Straight Path", new MecanumPathFollower(drive, "Straight Path", Constants.FastAutoConstraints, true));
    autoOptions.addOption("Turn 180 Degrees", new MecanumPathFollower(drive, "Turn 180 Degrees", Constants.SlowAutoConstraints, true));


    // Actual Pathing
    autoOptions.addOption("Three Piece Left", threePieceAutoLeft());
    autoOptions.addOption("Two Piece Left", twoPieceAutoLeft());
    autoOptions.addOption("Two Piece Left + Charge", twoPieceAutoChargeLeft());
    autoOptions.addOption("Two Piece Right", twoPieceAutoRight());
    autoOptions.addOption("Two Piece Right + Charge", twoPieceAutoChargeRight());

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

  public SequentialCommandGroup twoPieceAutoLeft() {
    return new SequentialCommandGroup(
       // new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station1)),
      //  new WaitCommand(1),
        new MecanumPathFollower(
            drive, "FirstPickUpLeftSide", Constants.MediumAutoConstraints, true),
        new WaitCommand(1),
        new MecanumPathFollower(
            drive, "FirstDropOffLeftSide", Constants.MediumAutoConstraints, false));
  }

  public SequentialCommandGroup twoPieceAutoChargeLeft() {
    return new SequentialCommandGroup(
        twoPieceAutoLeft(),
        new WaitCommand(1),
        new MecanumPathFollower(
            drive, "2pieceChargingLeft", Constants.MediumAutoConstraints, false));
  }

  public SequentialCommandGroup threePieceAutoLeft() {
    return new SequentialCommandGroup(
        twoPieceAutoLeft(),
        new MecanumPathFollower(
            drive, "SecondPickUpLeftSide", Constants.MediumAutoConstraints, false),
        new WaitCommand(1),
        new MecanumPathFollower(
            drive, "SecondDropOffLeftSide", Constants.MediumAutoConstraints, false));
  }

  public SequentialCommandGroup twoPieceAutoRight() {
    return new SequentialCommandGroup(
        new PathToPose(drive, () -> FieldUtil.getTransformPoseStation(FieldUtil.Station9)),
        new WaitCommand(1),
        new MecanumPathFollower(
            drive, "FirstPickUpRightSide", Constants.MediumAutoConstraints, true),
        new WaitCommand(1),
        new MecanumPathFollower(
            drive, "FirstDropOffRightSide", Constants.MediumAutoConstraints, false));
  }

  public SequentialCommandGroup twoPieceAutoChargeRight() {
    return new SequentialCommandGroup(
        twoPieceAutoRight(),
        new WaitCommand(1),
        new MecanumPathFollower(
            drive, "2pieceChargingRight", Constants.MediumAutoConstraints, false));
  }
}
