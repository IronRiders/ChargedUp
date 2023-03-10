package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MecanumWheel extends SubsystemBase {
  private final CANSparkMax motor;
  private RelativeEncoder encoder;
  private PIDController pidController;
  private static SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(0.13888, 2.2504, 0.45863);

  public MecanumWheel(int motorId, boolean inverted) {
    motor = new CANSparkMax(motorId, MotorType.kBrushless);
    motor.setInverted(inverted);
    motor.setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
    motor.setIdleMode(IdleMode.kBrake);
    pidController = new PIDController(0, 0, 0);
    encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(Constants.WHEEL_CIRCUMFERENCE / Constants.GEARING);
  }

  public void log() {
    SmartDashboard.putNumber("MecanumWheel/Max Linear Velocity", getMaxLinearVelocity());
    SmartDashboard.putNumber("MecanumWheel/Velocity", getVelocity());
    SmartDashboard.putNumber("Target Velocity", pidController.getSetpoint());
  }

  public double getVelocity() {
    return (encoder.getVelocity() / Constants.GEARING / 60 * Constants.WHEEL_CIRCUMFERENCE);
  }

  public void setVelocity(double mps, boolean needPID) {
    if (needPID) {
      motor.setVoltage(feedForward.calculate(mps) + pidController.calculate(getVelocity(), mps));
    } else {
      motor.setVoltage(feedForward.calculate(mps));
    }
  }

  public static double getMaxLinearVelocity() {
    return feedForward.maxAchievableVelocity(12.0, 0);
  }

  public double getWheelPostions() {
    return encoder.getPosition();
  }
}
