package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants; 

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax boxClimberMotor;
    private CANSparkMax armMotor;

    RelativeEncoder boxClimberMotorEncoder = boxClimberMotor.getEncoder();

    public ArmSubsystem() {
        int currentLimit = 10;

        boxClimberMotor = new CANSparkMax(Constants.ARM_PORT1, MotorType.kBrushless);
        boxClimberMotor.setIdleMode(IdleMode.kBrake);


        boxClimberMotor.setSmartCurrentLimit(currentLimit);

        armMotor = new CANSparkMax(Constants.ARM_PORT2, MotorType.kBrushless);
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor.setSmartCurrentLimit(currentLimit);
    }

    public double getEncoderDistance() {
        return boxClimberMotorEncoder.getPosition();
    }

    public void setBoxClimberMotor(double speed) {
        boxClimberMotor.set(speed);
        armMotor.set(speed);
    }

    public void stop() {
        boxClimberMotor.set(0);
        armMotor.set(0);
    } 


}
 
 /*public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax boxClimberMotor;
    private CANSparkMax armMotor;
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double setpoint;
    private double error = 0;
    private double prevError = 0;


    public final PIDController armPid = new PIDController(kP, kI, kD);
    Encoder encoder = new Encoder(DIO_PIN1, DIO_PIN2);
    
    public ArmSubsystem() {
        
        boxClimberMotor = new CANSparkMax(Constants.ARM_PORT1, MotorType.kBrushless);
        boxClimberMotor.setIdleMode(IdleMode.kBrake);
        boxClimberMotor.setSmartCurrentLimit(ARM_CURRENT_LIMIT);

        armMotor = new CANSparkMax(Constants.ARM_PORT2, MotorType.kBrushless);
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor.setSmartCurrentLimit(ARM_CURRENT_LIMIT);        
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        armPid.setSetpoint(setpoint);
    } 
    
    public void extend() {
        double output = armPid.calculate(encoder.getDistance(), setpoint);
        boxClimberMotor.set(output);
        armMotor.set(output);
    }

    public void retract() {
        double output = armPid.calculate(encoder.getDistance(), setpoint);
        boxClimberMotor.set(-output);
        armMotor.set(-output);
    }
    

    public void stop() {
        boxClimberMotor.set(0);
        armMotor.set(0);
    } 
} */
