package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.DriveSubsytem;
import frc.robot.subsystems.ExtendingArmSubsystem;
import frc.robot.subsystems.RaiseLowerArmSubsystem;

public class BurnFlashCommand extends CommandBase {
  private final ManipulatorSubsystem manipulatorSubsystem;
  private final DriveSubsytem driveSubsystem;
  private final ExtendingArmSubsystem extendingArmSubsystem;
  private final RaiseLowerArmSubsystem raiseLowerArmSubsystem;

  public BurnFlashCommand(
      DriveSubsytem driveSubsystem,
      ExtendingArmSubsystem extendingArmSubsystem,
      RaiseLowerArmSubsystem raiseLowerArmSubsystem,
      ManipulatorSubsystem manipulatorSubsystem) {
    this.manipulatorSubsystem = manipulatorSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.extendingArmSubsystem = extendingArmSubsystem;
    this.raiseLowerArmSubsystem = raiseLowerArmSubsystem;
    addRequirements(
        driveSubsystem,
        manipulatorSubsystem,
        extendingArmSubsystem,
        raiseLowerArmSubsystem,
        manipulatorSubsystem);
  }

  @Override
  public void initialize() {
    manipulatorSubsystem.burnFlash();
    driveSubsystem.burnFlash();
    extendingArmSubsystem.burnFlash();
    raiseLowerArmSubsystem.burnFlash();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
