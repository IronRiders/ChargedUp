package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
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

public class ArmSubsystem extends PIDSubsystem {
  private CANSparkMax motor;
  private final SimpleMotorFeedforward simpleMotorFeedforward = new SimpleMotorFeedforward(0, 1);

  public ArmSubsystem() {
    super(new PIDController(0, 0, 0), 0);
    motor = new CANSparkMax(Constants.ARM_CLIMBER_PORT, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(Constants.ARM_CURRENT_LIMIT);
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    double feedforward = simpleMotorFeedforward.calculate(8, 0);
   motor.setVoltage(output + feedforward);
  }
  
  public void extend() {
    motor.set(0.2);
  }

  public void retract() {
    motor.set(-0.2);
  }

  public void stop() {
    motor.set(0);
  }

  public void burnFlash() {
    motor.burnFlash();
  }

  @Override
  protected double getMeasurement() {
    // TODO Auto-generated method stub
    return 0;
  }
}
