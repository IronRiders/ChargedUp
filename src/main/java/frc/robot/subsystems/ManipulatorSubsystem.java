package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase {
  private CANSparkMax manipulatorMotor1;
  private CANSparkMax manipulatorMotor2;

  RelativeEncoder manipulatorMotor1Encoder;
  RelativeEncoder manipulatorMotor2Encoder;

  boolean motorRunning = false;

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
    if (motorRunning) {
      double motor1Velocity = manipulatorMotor1Encoder.getVelocity();
      double motor2Velocity = manipulatorMotor2Encoder.getVelocity();
      if ((Math.abs(motor1Velocity) >= Constants.STALL_SPEED) || (Math.abs(motor2Velocity) >= Constants.STALL_SPEED)) {
        // The motor is stalled
        stop();
      }
    }
  }

  public void stop() {
    setManipulatorMotors(0);
    motorRunning = false;
  }

  public void setManipulatorMotors(double speed) {
    manipulatorMotor1.set(speed);
    manipulatorMotor2.set(speed);
    motorRunning = true;
  }

  public void release() {
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
