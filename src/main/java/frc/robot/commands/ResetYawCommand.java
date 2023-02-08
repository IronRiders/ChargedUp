package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MecanumDrive;

public class ResetYawCommand extends CommandBase {
  private final MecanumDrive mecanumDrive;

  public ResetYawCommand(MecanumDrive mecanumDrive) {
    this.mecanumDrive = mecanumDrive;
    addRequirements(mecanumDrive);
  }

  @Override
  public void initialize() {
    mecanumDrive.pigeon.setYaw(0);
    this.end(false);
  }
}
