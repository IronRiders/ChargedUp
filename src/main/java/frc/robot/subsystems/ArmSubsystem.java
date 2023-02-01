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

  public ArmSubsystem() {
    int currentLimit = 10;

    boxClimberMotor = new CANSparkMax(Constants.ARM_PORT1, MotorType.kBrushless);
    boxClimberMotor.setIdleMode(IdleMode.kBrake);
    boxClimberMotor.setSmartCurrentLimit(currentLimit);

    armMotor = new CANSparkMax(Constants.ARM_PORT2, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setSmartCurrentLimit(currentLimit);
    }

    public double getEncoderDistance() {
        return boxClimberMotorEncoder.getPosition();
    }

    public void setBoxClimberMotor(double speed) {
        boxClimberMotor.set(speed);
    }

    public void setArmMotor(double speed) {
        armMotor.set(speed);
    }

    public void stop() {
        boxClimberMotor.set(0);
        armMotor.set(0);
    } 
}
