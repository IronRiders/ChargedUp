package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
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
  PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
  private CANSparkMax motor;
  private int clockCounter = 0;
  private boolean hasHit = false;

  public ArmSubsystem() {
    motor = new CANSparkMax(Constants.ARM_CLIMBER_PORT, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(Constants.ARM_CURRENT_LIMIT);

  }

  @Override
  public void periodic() {
    if(pdh.getCurrent(3)>Constants.STALL_CURRENT){
      
      if (hasHit && clockCounter< 5){
        stop();
        hasHit = false;
      } else if (hasHit && clockCounter <= 5){
        Reverse();
      } else {
        hasHit = true;
      }
    }
    clockCounter += 1;
  }

  public void extend() {
    clockCounter = 0;
    motor.set(Constants.Arm_POWER);
  }

  public void retract() {
    clockCounter = 0;
    motor.set(-Constants.Arm_POWER);
  }

  public void Reverse(){
    if (motor.getInverted()) {
      motor.setInverted(false);
    } else {
      motor.setInverted(true);
    }
  }

  public void stop() {
    motor.set(0);
  }

  public void burnFlash() {
    motor.burnFlash();
  }
}
