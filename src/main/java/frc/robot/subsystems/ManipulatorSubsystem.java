package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase {
  private CANSparkMax manipulatorMotor1;


  RelativeEncoder manipulatorMotor1Encoder;

  public ManipulatorSubsystem() {
    manipulatorMotor1 = new CANSparkMax(Constants.MANIPULATOR_PORT1, MotorType.kBrushless); //Deleted motor two because the new maniupaltor only use one motor and just changes the way it spin to decide what type of object it wants to pick up
  
    manipulatorMotor1.setIdleMode(IdleMode.kBrake);
    manipulatorMotor1.setSmartCurrentLimit(Constants.MANIPULATOR_CURRENT_LIMIT);

    manipulatorMotor1Encoder = manipulatorMotor1.getEncoder(); // IDK if we need a encoder so that needs to be worked out
  }

  public void grab(GrabObject grabObject) {
    switch (grabObject) {
      case CONE:
        //manipulatorMotor1.set(-Constants.MANIPULATOR_SPEED_CONE); // This might be faster because the set manipulator motors was for two motors but now we only have one for the new intake
        setManipulatorMotors(-Constants.MANIPULATOR_SPEED_CONE); //This might be to pick up the cone or cube and needs to be tested
        break;

      case BOX:
        //manipulatorMotor1.set(speed); // Same as above
        setManipulatorMotors(Constants.MANIPULATOR_SPEED_BOX);
        break;
    }
  }

  public void release() {
    manipulatorMotor1.set(-Constants.MANIPULATOR_SPEED_CONE);
  }

  public double getManipulatorMotor1EncoderDistance() { // we probably won't need the encoder any more because that was to measure the distance for how apart the intake pieces were but with the rollers we wouldn't need to do that
    return manipulatorMotor1Encoder.getPosition();
  }

  public void resetManipulatorMotor1EncoderDistance() {
    manipulatorMotor1Encoder.setPosition(0);
  }

  public void setManipulatorMotors(double speed) {
    manipulatorMotor1.set(speed);
  }

  // Create a method that stops the climbers from moving
  public void stop() {
    manipulatorMotor1.set(0);
  }
}
