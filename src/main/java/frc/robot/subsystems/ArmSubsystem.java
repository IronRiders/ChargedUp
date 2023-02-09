package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax boxClimberMotor;
  private CANSparkMax armMotor;

  RelativeEncoder boxClimberMotorEncoder = boxClimberMotor.getEncoder();
  RelativeEncoder armMotorEncoder = armMotor.getEncoder();

  public ArmSubsystem() {
    int currentLimit = 10;

    boxClimberMotor = new CANSparkMax(Constants.ARM_BOX_CLIMBER_PORT, MotorType.kBrushless);
    boxClimberMotor.setIdleMode(IdleMode.kBrake);
    boxClimberMotor.setSmartCurrentLimit(currentLimit);

    armMotor = new CANSparkMax(Constants.ARM_RAISE_LOWER_PORT, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setSmartCurrentLimit(currentLimit);
  }

  public double getBoxClimberEncoderDistance() {
    return boxClimberMotorEncoder.getPosition();
  }

  public double getArmMotorEncoderDistance() {
    return armMotorEncoder.getPosition();
  }

  public void setBoxClimberMotor(double speed) {
    boxClimberMotor.set(speed);
  }

  public void setArmMotor(double speed) {
    armMotor.set(speed);
  }

  public void extend() {
    boxClimberMotor.set(Constants.BOX_CLIMBER_MOTOR_POWER);
  }

  public void retract() {
    boxClimberMotor.set(-Constants.BOX_CLIMBER_MOTOR_POWER);
  }

  public void raise() {
    armMotor.set(Constants.ARM_MOTOR_POWER);
  }

  public void lower() {
    armMotor.set(-Constants.ARM_MOTOR_POWER);
  }

  public void stop() {
    boxClimberMotor.set(0);
    armMotor.set(0);
  }
}
