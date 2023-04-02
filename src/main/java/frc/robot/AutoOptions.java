package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoLevelingCommand;
import frc.robot.commands.ForwardCommand;
import frc.robot.commands.MecanumPathFollower;
import frc.robot.commands.PathToPose;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.util.FieldUtil;

public class AutoOptions {
  private SendableChooser<CommandBase> autoOptions = new SendableChooser<>();
  private DriveSubsystem drive;
  private PivotSubsystem pivot;
  private ArmSubsystem arm;
  private ManipulatorSubsystem manipulator;

  public AutoOptions(
      DriveSubsystem drive,
      PivotSubsystem pivot,
      ArmSubsystem arm,
      ManipulatorSubsystem manipulator) {
    this.drive = drive;
    this.pivot = pivot;
    this.arm = arm;
    this.manipulator = manipulator;

    // Tuning
    /*  autoOptions.setDefaultOption(
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
    autoOptions.addOption(
        "Straight Path",
        new MecanumPathFollower(drive, "Straight Path", Constants.FastAutoConstraints, true));
    autoOptions.addOption(
        "Turn 180 Degrees",
        new MecanumPathFollower(drive, "Turn 180 Degrees", Constants.SlowAutoConstraints, true));

    // Actual Pathing
    autoOptions.addOption("Three Piece Left", threePieceAutoLeft());
    autoOptions.addOption("Two Piece Left", twoPieceAutoLeft());
    autoOptions.addOption("Two Piece Left + Charge", twoPieceAutoChargeLeft());
    autoOptions.addOption("Two Piece Right", twoPieceAutoRight());
    autoOptions.addOption("Two Piece Right + Charge", twoPieceAutoChargeRight());*/

    // PATHS TO USE AT AUBURN
    autoOptions.setDefaultOption("Place Cone and Balance", PlaceAndbalance("cone"));
    autoOptions.setDefaultOption("Place Cube and Balance", PlaceAndbalance("cube"));
    autoOptions.addOption("Place Cube And Back UP", PlaceAndRunBack("cube"));
    autoOptions.addOption("Place Cone And Back UP", PlaceAndRunBack("cone"));

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
        // new PathToPose(drive, () ->
        // FieldUtil.getTransformPoseStation(FieldUtil.Station1)),
        // new WaitCommand(1),
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

  public SequentialCommandGroup PlaceAndbalance(String object) {
    return new SequentialCommandGroup(
        (object.equalsIgnoreCase("cone") ? L1AutoArmMove() : L1AutoArmMove()),
        new WaitCommand(0.75),
        new ForwardCommand(drive, Units.feetToMeters(5.5)),
       new AutoLevelingCommand(drive));
  }

  public SequentialCommandGroup PlaceAndRunBack(String object) {
    return new SequentialCommandGroup(
        (object.equalsIgnoreCase("cone") ? L1AutoArmMove() : L1AutoArmMove()),
        new WaitCommand(2),
        new ForwardCommand(drive, Units.feetToMeters(5)));
  }

  public SequentialCommandGroup L3CubeAutoArmMove() {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              pivot.setGoal(Units.degreesToRadians(Constants.L3ANGLE));
              pivot.enable();
            },
            pivot),
        new WaitCommand(1.25),
 //       new StartEndCommand(arm::extend, arm::stop, arm).withTimeout(1.35),
        new StartEndCommand(manipulator::release, manipulator::stop, manipulator).withTimeout(0.2),
  //      new StartEndCommand(arm::retract, arm::stop, arm).withTimeout(1.35),
        new InstantCommand(
            () -> {
              pivot.setGoal(Units.degreesToRadians(30));
              pivot.enable();
            },
            pivot));
  }

  public SequentialCommandGroup L2CubeAutoArmMove() {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              pivot.setGoal(Units.degreesToRadians(Constants.L3ANGLE));
              pivot.enable();
            },
            pivot),
        new WaitCommand(1.15),
        new StartEndCommand(manipulator::release, manipulator::stop, manipulator).withTimeout(0.2),
 //       new StartEndCommand(arm::retract, arm::stop, arm).withTimeout(1.25),
        new InstantCommand(
            () -> {
              pivot.setGoal(Units.degreesToRadians(30));
              pivot.enable();
            },
            pivot));
  }

  public SequentialCommandGroup L1AutoArmMove() {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              pivot.setGoal(Units.degreesToRadians(Constants.L1ANGLE));
              pivot.enable();
            },
            pivot),
        new WaitCommand(1),
        new StartEndCommand(manipulator::release, manipulator::stop, manipulator).withTimeout(0.2),
        new InstantCommand(
            () -> {
              pivot.setGoal(Units.degreesToRadians(30));
              pivot.enable();
            },
            pivot));
  }

  public SequentialCommandGroup L3ConeAutoArmMove() {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              pivot.setGoal(Units.degreesToRadians(Constants.L3ANGLE));
              pivot.enable();
            },
            pivot),
        new WaitCommand(2),
      //  new StartEndCommand(arm::extend, arm::stop, arm).withTimeout(1.5),
        new StartEndCommand(manipulator::release, manipulator::stop, manipulator).withTimeout(0.2),
     //   new StartEndCommand(arm::retract, arm::stop, arm).withTimeout(1.5),
        new InstantCommand(
            () -> {
              pivot.setGoal(Units.degreesToRadians(30));
              pivot.enable();
            },
            pivot));
  }

  public SequentialCommandGroup L2ConeAutoArmMove() {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              pivot.setGoal(Units.degreesToRadians(Constants.L2ANGLE));
              pivot.enable();
            },
            pivot),
        new WaitCommand(2),
     //   new StartEndCommand(arm::extend, arm::stop, arm).withTimeout(0.2),
        new StartEndCommand(manipulator::release, manipulator::stop, manipulator).withTimeout(0.2),
   //     new StartEndCommand(arm::retract, arm::stop, arm).withTimeout(0.3),
        new InstantCommand(
            () -> {
              pivot.setGoal(Units.degreesToRadians(30));
              pivot.enable();
            },
            pivot));
  }

  public SequentialCommandGroup loadingStationAutoArmMoveUp() {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              pivot.setGoal(Units.degreesToRadians(Constants.LHUMAN));
              pivot.enable();
            },
            pivot),
        new WaitCommand(2));
     //   new StartEndCommand(arm::extend, arm::stop, arm).withTimeout(1.5));
  }

  public SequentialCommandGroup loadingStationAutoArmMoveDown() {
    return new SequentialCommandGroup(
     //   new StartEndCommand(arm::retract, arm::stop, arm).withTimeout(1.75),
        new InstantCommand(
            () -> {
              pivot.setGoal(Units.degreesToRadians(30));
              pivot.enable();
            },
            pivot));
  }

  public SequentialCommandGroup GroundPickUp() {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              pivot.setGoal(Units.degreesToRadians(Constants.LGROUND));
              pivot.enable();
            },
            pivot));
  }

  public SequentialCommandGroup GroundDropOff() {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              pivot.setGoal(Units.degreesToRadians(Constants.ARM_OFF_SET_RADS));
              pivot.enable();
            },
            pivot));
  }
}
