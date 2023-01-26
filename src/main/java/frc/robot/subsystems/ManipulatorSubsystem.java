package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase {
  private CANSparkMax manipulatorMotor1;
  private CANSparkMax manipulatorMotor2;

  public ManipulatorSubsystem() {
    manipulatorMotor1 = new CANSparkMax(Constants.MANIPULATOR_PORT1, MotorType.kBrushless);
    manipulatorMotor2 = new CANSparkMax(Constants.MANIPULATOR_PORT2, MotorType.kBrushless);

    manipulatorMotor1.setIdleMode(IdleMode.kBrake);
    manipulatorMotor1.setSmartCurrentLimit(Constants.MANIPULATOR_CURRENT_LIMIT);

    manipulatorMotor2.setIdleMode(IdleMode.kBrake);
    manipulatorMotor2.setSmartCurrentLimit(Constants.MANIPULATOR_CURRENT_LIMIT);
  }

  public void grab() {
    manipulatorMotor1.set(Constants.MANIPULATOR_POWER);
    manipulatorMotor2.set(-Constants.MANIPULATOR_POWER);
  }

  public void release() {
    manipulatorMotor1.set(-Constants.MANIPULATOR_POWER);
    manipulatorMotor2.set(Constants.MANIPULATOR_POWER);
  }

  // Create a method that stops the climbers from moving
  public void stop() {
    manipulatorMotor1.set(0);
    manipulatorMotor2.set(0);
  }
}
