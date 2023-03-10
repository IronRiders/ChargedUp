package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax motor;
    public double currentHeight = 0;
    public double minExtension = Constants.maximumExtenstionHeight; // All in inches
    public double maxExtension = Constants.minimumExtensionHeight;
    public double speed = 1.0;

    public ArmSubsystem() {
        motor = new CANSparkMax(Constants.ARM_BOX_CLIMBER_PORT, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(Constants.BOX_CLIMBER_MOTOR_CURRENT_LIMIT);

        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        motor.setSoftLimit(SoftLimitDirection.kForward, Constants.maximumExtenstionHeight);
        motor.setSoftLimit(SoftLimitDirection.kReverse, Constants.minimumExtensionHeight);

        motor.getPIDController().setFF(0);
        motor.getPIDController().setI(0);
        motor.getPIDController().setP(Constants.ARM_KP);

        motor.getEncoder().setPositionConversionFactor(Units.degreesToRadians(360.0) / Constants.Arm_GEAR_RATIO); // In Degrees
                                                                                                                  
        motor.getEncoder()
                .setVelocityConversionFactor((Units.degreesToRadians(360.0) / Constants.Arm_GEAR_RATIO) / 60.0); // Degrees Per Second
    }

    public void setTelescopeSpeed(double extension, boolean isPositiveSpeed) {
        maxExtension = isPositiveSpeed ? extension : Constants.minimumExtensionHeight;
        minExtension = !isPositiveSpeed ? extension : Constants.minimumExtensionHeight;
        if (motor.getEncoder().getPosition() == maxExtension) {
            motor.set(0);
            return;
        }
        motor.set(isPositiveSpeed ? speed : -speed);
    }

    public void periodic() {
        setHeight(currentHeight, minExtension, maxExtension);
    }

    public void resetTelescopeEncoder() {
        motor.getEncoder().setPosition(0);
    }

    public double getHeight() {
        return motor.getEncoder().getPosition();
    }

    public void setTargetHeight(double height) {
        currentHeight = height;
    }

    public double getTargetHeight() {
        return currentHeight;
    }

    private void setHeight(double height, double minExtenstion, double maxExtension) {
        if (height < minExtenstion || height > maxExtension)
            return;
        motor.getPIDController().setReference(height, ControlType.kPosition);
    }
}*/

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax motor;
  
    public ArmSubsystem() {
      motor = new CANSparkMax(Constants.ARM_CLIMBER_PORT, MotorType.kBrushless);
      motor.setIdleMode(IdleMode.kBrake);
      motor.setSmartCurrentLimit(Constants.ARM_CURRENT_LIMIT);
    }
  
    public void extend() {
      motor.set(Constants.Arm_POWER);
    }
  
    public void retract() {
      motor.set(-Constants.Arm_POWER);
    }
  
    public void stop() {
      motor.set(0);
    }
  }
  
