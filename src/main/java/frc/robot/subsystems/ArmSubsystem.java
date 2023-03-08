package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax boxClimberMotor;
  private CANSparkMax armMotor;

  RelativeEncoder boxClimberMotorEncoder;
  RelativeEncoder armMotorEncoder;
  SparkMaxLimitSwitch limitSwitch;

  public ArmSubsystem() {
    boxClimberMotor = new CANSparkMax(Constants.ARM_BOX_CLIMBER_PORT, MotorType.kBrushless);
    boxClimberMotor.setIdleMode(IdleMode.kBrake);
    boxClimberMotor.setSmartCurrentLimit(Constants.BOX_CLIMBER_MOTOR_CURRENT_LIMIT);

    armMotor = new CANSparkMax(Constants.ARM_RAISE_LOWER_PORT, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setSmartCurrentLimit(Constants.ARM_MOTOR_CURRENT_LIMIT);
    limitSwitch = armMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    limitSwitch.enableLimitSwitch(true);

    boxClimberMotorEncoder = boxClimberMotor.getEncoder();
    armMotorEncoder = armMotor.getEncoder();
  }

  public void resetBoxClimberMotorEncoder() {
    boxClimberMotorEncoder.setPosition(0);
  }

  public void resetArmMotorEncoder() {
    armMotorEncoder.setPosition(0);
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
