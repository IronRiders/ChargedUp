package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(Constants.ARM_CLIMBER_PORT, MotorType.kBrushless);
    private final RelativeEncoder encoder;
    private final ProfiledPIDController PIDController = new ProfiledPIDController(Constants.ARM_KP, 0, 0, Constants.kConstraints);

    private double targetVolts = 0;
    private double targetRotations = 0;

    public ArmSubsystem() {
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kBrake);
        motor.enableVoltageCompensation(12.0);
        motor.setSmartCurrentLimit(10);

        encoder = motor.getEncoder();
        encoder.setPosition(0);
    }

    @Override
    public void periodic() {
        double adjustedVoltage = PIDController.calculate(encoder.getPosition(), targetRotations);
        if(encoder.getPosition() >= 144) adjustedVoltage = Math.min(adjustedVoltage, 0);
        if(encoder.getPosition() <= 0) adjustedVoltage = Math.max(adjustedVoltage, 0);

        motor.setVoltage(adjustedVoltage);
    }

    public void stop() {
        motor.setVoltage(0);
    }

    public void setVolts(double voltage){
        targetVolts = voltage;
    }

    public void setRotations(double rotations){
            PIDController.reset(encoder.getPosition(), encoder.getVelocity());
            targetRotations = rotations;
    }

    public double getCurrentDraw(){
        return motor.getOutputCurrent();
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
  
