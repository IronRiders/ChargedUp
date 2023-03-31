package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class WristSubsystem extends ProfiledPIDSubsystem {
    private final CANSparkMax motor  = new CANSparkMax(Constants.PIVOT_PORT, MotorType.kBrushless);
    private final ArmFeedforward wristFeedforward =
    new ArmFeedforward(
        Constants.PIVOT_KS, Constants.PIVOT_KGR, Constants.PIVOT_KV, Constants.PIVOT_KA);

    public WristSubsystem() {
      super(
          new ProfiledPIDController(
              Constants.WRIST_KP,
              0,
              0,
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(Constants.WRIST_VELOCITY_DEG),
                  Units.degreesToRadians(Constants.WRIST_ACCELERATION_DEG))),
          0);
      motor.setIdleMode(IdleMode.kBrake);
      motor.setSmartCurrentLimit(Constants.PIVOT_CURRENT_LIMIT);
      motor.enableVoltageCompensation(12);
      motor.setSoftLimit(SoftLimitDirection.kForward, 2.0f);
  
      motor
          .getEncoder()
          .setPositionConversionFactor(
              Units.degreesToRadians(360.0) / Constants.WRIST_GEAR_RATIO); // In Degrees
      motor
          .getEncoder()
          .setVelocityConversionFactor(
              (Units.degreesToRadians(360.0) / Constants.WRIST_GEAR_RATIO)
                  / 60.0); // Degrees Per Second
  
      setGoal(Constants.WRIST_OFF_SET_RADS);
    }
  
    @Override
    protected void useOutput(double output, State setpoint) {
      double feedforward = wristFeedforward.calculate(setpoint.position, setpoint.velocity);
      motor.setVoltage(output + feedforward);
    }
  
    @Override
    protected double getMeasurement() {
      return motor.getEncoder().getPosition() + Constants.ARM_OFF_SET_RADS;
    }
  
    public void burnFlash() {
      motor.burnFlash();
    }
}
