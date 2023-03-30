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
  private final CANSparkMax manipulatorMotor;

  RelativeEncoder manipulatorMotorEncoder;
  PowerDistribution pdh = new PowerDistribution(13, ModuleType.kRev);

  public ManipulatorSubsystem() {
    manipulatorMotor = new CANSparkMax(Constants.MANIPULATOR_PORT, MotorType.kBrushed);

    manipulatorMotor.setIdleMode(IdleMode.kBrake);
    manipulatorMotor.setSmartCurrentLimit(Constants.MANIPULATOR_CURRENT_LIMIT);

    manipulatorMotorEncoder = manipulatorMotor.getEncoder();
  }

  public void grab(GrabObject grabObject) {
    switch (grabObject) {
      case CONE:
        setManipulatorMotors(Constants.MANIPULATOR_SPEED_CONE);
        manipulatorMotor.setInverted(false);
        break;
      case BOX:
        setManipulatorMotors(Constants.MANIPULATOR_SPEED_BOX);
        manipulatorMotor.setInverted(true);
        break;
    }
  }

  public void expel(GrabObject grabObject) {
    switch (grabObject) {
      case CONE:
        setManipulatorMotors(Constants.MANIPULATOR_SPEED_CONE);
        manipulatorMotor.setInverted(true);
        break;
      case BOX:
        setManipulatorMotors(Constants.MANIPULATOR_SPEED_BOX);
        manipulatorMotor.setInverted(false);
        break;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("17:", pdh.getCurrent(16));
  }

  public void setManipulatorMotors(double speed) {
    manipulatorMotor.set(speed);
  }

  public void burnFlash() {
    manipulatorMotor.burnFlash();
  }
}
