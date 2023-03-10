package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ExtendingArmSubsystem extends SubsystemBase {
  private CANSparkMax boxClimberMotor;
  public double extendingPosition;

  RelativeEncoder boxClimberMotorEncoder;
  SparkMaxLimitSwitch limitSwitch;

  public final RaiseLowerArmSubsystem armRaise = new RaiseLowerArmSubsystem();

  public ExtendingArmSubsystem() {
    boxClimberMotor = new CANSparkMax(Constants.ARM_BOX_CLIMBER_PORT, MotorType.kBrushless);
    boxClimberMotor.setIdleMode(IdleMode.kBrake);
    boxClimberMotor.setSmartCurrentLimit(Constants.BOX_CLIMBER_MOTOR_CURRENT_LIMIT);
    extendingPosition = getBoxClimberEncoderDistance() * Constants.ARM_MOTOR_CIRCUMFERENCE;

    boxClimberMotorEncoder = boxClimberMotor.getEncoder();
  }

  public void resetBoxClimberMotorEncoder() {
    boxClimberMotorEncoder.setPosition(0);
  }

  public double getBoxClimberEncoderDistance() {
    return boxClimberMotorEncoder.getPosition();
  }

  public void setBoxClimberMotor(double speed) {
    boxClimberMotor.set(speed);
  }

  public void extend() {
    if (extendingPosition <= Constants.EXTENDING_ARM_LIMIT
        && armRaise.raisePosition <= Constants.RAISE_LOWER_ARM_LIMIT) {
      boxClimberMotor.set(Constants.BOX_CLIMBER_MOTOR_POWER);
    }
  }

  public void retract() {
    boxClimberMotor.set(-Constants.BOX_CLIMBER_MOTOR_POWER);
  }

  public void stop() {
    boxClimberMotor.set(0);
  }
}
