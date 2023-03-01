package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class RaiseLowerArmSubsystem extends SubsystemBase {
  private CANSparkMax armMotor;

  RelativeEncoder armMotorEncoder;

  public RaiseLowerArmSubsystem() {

    armMotor = new CANSparkMax(Constants.ARM_RAISE_LOWER_PORT, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setSmartCurrentLimit(Constants.ARM_MOTOR_CURRENT_LIMIT);

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
    armMotor.set(Constants.ARM_MOTOR_POWER);
  }

  public void lower() {
    armMotor.set(-Constants.ARM_MOTOR_POWER);
  }

  public void stop() {
    armMotor.set(0);
  }
}


