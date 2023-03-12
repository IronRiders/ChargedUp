package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class BurnFlashCommand extends CommandBase {
  private final ManipulatorSubsystem manipulatorSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final ArmSubsystem armSubsystem;

  public BurnFlashCommand(
      DriveSubsystem driveSubsystem,
      PivotSubsystem pivotSubsystem,
      ArmSubsystem armSubsystem,
      ManipulatorSubsystem manipulatorSubsystem) {
    this.manipulatorSubsystem = manipulatorSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.armSubsystem = armSubsystem;
    addRequirements(
        driveSubsystem,
        manipulatorSubsystem,
        pivotSubsystem,
        armSubsystem,
        manipulatorSubsystem);
  }

  @Override
  public void initialize() {
    manipulatorSubsystem.burnFlash();
    driveSubsystem.burnFlash();
    pivotSubsystem.burnFlash();
    armSubsystem.burnFlash();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
