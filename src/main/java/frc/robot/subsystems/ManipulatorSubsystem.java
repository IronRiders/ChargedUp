package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase {
  private CANSparkMax manipulatorMotor1;
  private CANSparkMax manipulatorMotor2;

  RelativeEncoder manipulatorMotor1Encoder = manipulatorMotor1.getEncoder();

  public ManipulatorSubsystem() {
    manipulatorMotor1 = new CANSparkMax(Constants.MANIPULATOR_PORT1, MotorType.kBrushless);
    manipulatorMotor2 = new CANSparkMax(Constants.MANIPULATOR_PORT2, MotorType.kBrushless);

    manipulatorMotor1.setIdleMode(IdleMode.kBrake);
    manipulatorMotor1.setSmartCurrentLimit(Constants.MANIPULATOR_CURRENT_LIMIT);

    manipulatorMotor2.setIdleMode(IdleMode.kBrake);
    manipulatorMotor2.setSmartCurrentLimit(Constants.MANIPULATOR_CURRENT_LIMIT);
  }

  public void setGrabCurrentLimit(GrabObject grabObject) {
    switch (grabObject) {
      case CONE:
        manipulatorMotor1.setSmartCurrentLimit(Constants.MANIPULATOR_CURRENT_LIMIT_CONE);
        manipulatorMotor2.setSmartCurrentLimit(Constants.MANIPULATOR_CURRENT_LIMIT_CONE);
        break;

      case BOX:
        manipulatorMotor1.setSmartCurrentLimit(Constants.MANIPULATOR_CURRENT_LIMIT_BOX);
        manipulatorMotor2.setSmartCurrentLimit(Constants.MANIPULATOR_CURRENT_LIMIT_BOX);
        break;
    }
  }

  public void grab(GrabObject object) {
    setGrabCurrentLimit(object);
    manipulatorMotor1.set(Constants.MANIPULATOR_POWER);
    manipulatorMotor2.set(-Constants.MANIPULATOR_POWER);
  }

  public void release() {
    manipulatorMotor1.set(-Constants.MANIPULATOR_POWER);
    manipulatorMotor2.set(Constants.MANIPULATOR_POWER);
  }

  public double getManipulatorMotor1EncoderDistance() {
    return manipulatorMotor1Encoder.getPosition();
  }

  public void resetManipulatorMotor1EncoderDistance() {
    manipulatorMotor1Encoder.setPosition(0);
  }

  public void setManipulatorMotors(double speed) {
    manipulatorMotor1.set(speed);
    manipulatorMotor2.set(-speed);
  }

  public void setManipulatorMotors(double speed, GrabObject object) {
    setGrabCurrentLimit(object);
    manipulatorMotor1.set(speed);
    manipulatorMotor2.set(-speed);
  }

  // Create a method that stops the climbers from moving
  public void stop() {
    manipulatorMotor1.set(0);
    manipulatorMotor2.set(0);
  }
}
