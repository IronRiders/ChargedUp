package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase {
  private CANSparkMax manipulatorMotor1;
  private CANSparkMax manipulatorMotor2;

  RelativeEncoder manipulatorMotor1Encoder;
  RelativeEncoder manipulatorMotor2Encoder;
  PowerDistribution pdh = new PowerDistribution(13, ModuleType.kRev);
  boolean motorRunning = false;
  boolean motorBackwards = false;
  boolean hasHit = false;

  public ManipulatorSubsystem() {
    manipulatorMotor1 = new CANSparkMax(Constants.MANIPULATOR_PORT1, MotorType.kBrushless);
    manipulatorMotor2 = new CANSparkMax(Constants.MANIPULATOR_PORT2, MotorType.kBrushless);

    manipulatorMotor1.setIdleMode(IdleMode.kBrake);
    manipulatorMotor1.setInverted(true);
    manipulatorMotor1.setSmartCurrentLimit(Constants.MANIPULATOR_CURRENT_LIMIT);

    manipulatorMotor2.setIdleMode(IdleMode.kBrake);
    manipulatorMotor2.setSmartCurrentLimit(Constants.MANIPULATOR_CURRENT_LIMIT);

    manipulatorMotor1Encoder = manipulatorMotor1.getEncoder();
    manipulatorMotor2Encoder = manipulatorMotor2.getEncoder();
  }

  public void grab(GrabObject grabObject) {
    switch (grabObject) {
      case CONE:
        setManipulatorMotors(Constants.MANIPULATOR_SPEED_CONE);
        break;

      case BOX:
        setManipulatorMotors(Constants.MANIPULATOR_SPEED_BOX);
        break;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("17:", pdh.getCurrent(16));
    if (motorRunning && !motorBackwards) {
      // if (pdh.getCurrent(16) > Constants.STALL_CURRENT) {
      //   stop();
      //   motorRunning = false;
      // }
     // double motor1Velocity = manipulatorMotor1Encoder.getVelocity();
     // double motor2Velocity = manipulatorMotor2Encoder.getVelocity();
      // if ((Math.abs(motor1Velocity) >= Constants.STALL_SPEED) || (Math.abs(motor2Velocity) >=
      // Constants.STALL_SPEED)) {
      if (pdh.getCurrent(16) > Constants.STALL_CURRENT) {
        // The motor is stalled
        if (hasHit) {
          stop();
          hasHit = false;
        } else {
          hasHit = true;
        }
      }
    }
  }

  public void stop() {
    setManipulatorMotors(0.01);
    motorBackwards = false;
    motorRunning = false;
  }

  public void setManipulatorMotors(double speed) {
    motorRunning = true;
    manipulatorMotor1.set(speed);
    manipulatorMotor2.set(speed);
  }

  public void release() {
    motorBackwards = true;
    setManipulatorMotors(-Constants.MANIPULATOR_SPEED_CONE);
  }

  public double getManipulatorMotor1EncoderDistance() {
    return manipulatorMotor1Encoder.getPosition();
  }

  public void resetManipulatorMotor1EncoderDistance() {
    manipulatorMotor1Encoder.setPosition(0);
  }

  public void burnFlash() {
    manipulatorMotor1.burnFlash();
    manipulatorMotor2.burnFlash();
  }
}
