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

  RelativeEncoder manipulatorMotor1Encoder;

  public ManipulatorSubsystem() {
    manipulatorMotor1 = new CANSparkMax(Constants.MANIPULATOR_PORT1, MotorType.kBrushless);
    manipulatorMotor2 = new CANSparkMax(Constants.MANIPULATOR_PORT2, MotorType.kBrushless);

    manipulatorMotor1.setIdleMode(IdleMode.kBrake);
    manipulatorMotor1.setInverted(true);
    manipulatorMotor1.setSmartCurrentLimit(Constants.MANIPULATOR_CURRENT_LIMIT);

    manipulatorMotor2.setIdleMode(IdleMode.kBrake);
    manipulatorMotor2.setSmartCurrentLimit(Constants.MANIPULATOR_CURRENT_LIMIT);

    manipulatorMotor1Encoder = manipulatorMotor1.getEncoder();
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

  public void stop() {
    setManipulatorMotors(0);
  }

  public void setManipulatorMotors(double speed) {
    // manipulatorMotor1.set(speed);
    manipulatorMotor2.set(speed);
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
}
