package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class RaiseLowerArmSubsystem extends SubsystemBase {
  private CANSparkMax armMotor;
  public double raisePosition;

  RelativeEncoder armMotorEncoder;
  SparkMaxLimitSwitch limitSwitch;

  public final ExtendingArmSubsystem armExtend = new ExtendingArmSubsystem();

  public RaiseLowerArmSubsystem() {
    armMotor = new CANSparkMax(Constants.ARM_RAISE_LOWER_PORT, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setSmartCurrentLimit(Constants.ARM_MOTOR_CURRENT_LIMIT);
    limitSwitch = armMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    limitSwitch.enableLimitSwitch(true);
    raisePosition = getArmMotorEncoderDistance() * Constants.ARM_MOTOR_CIRCUMFERENCE;

    armMotorEncoder = armMotor.getEncoder();
  }

  public void resetArmMotorEncoder() {
    armMotorEncoder.setPosition(0);
  }

  public double getArmMotorEncoderDistance() {
    return armMotorEncoder.getPosition();
  }

  public void setArmMotor(double speed) {
    armMotor.set(speed);
  }

  public void raise() {
    if (armExtend.extendingPosition <= Constants.EXTENDING_ARM_LIMIT
        && raisePosition <= Constants.RAISE_LOWER_ARM_LIMIT) {
      armMotor.set(Constants.ARM_MOTOR_POWER);
    }
  }

  public void lower() {
    armMotor.set(-Constants.ARM_MOTOR_POWER);
  }

  public void stop() {
    armMotor.set(0);
  }
}
