package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class PivotSubsystem extends ProfiledPIDSubsystem {
  private CANSparkMax motor;
  private final ArmFeedforward armFeedforward =
      new ArmFeedforward(
          Constants.PIVOT_KS, Constants.PIVOT_KGR, Constants.PIVOT_KV, Constants.PIVOT_KA);

  public PivotSubsystem() {
    super(
        new ProfiledPIDController(
            Constants.Pivot_KP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(Constants.SHOULDER_VELOCITY_DEG),
                Units.degreesToRadians(Constants.SHOULDER_ACCELERATION_DEG))),
        0);
    motor = new CANSparkMax(Constants.PIVOT_PORT, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(Constants.PIVOT_CURRENT_LIMIT);
    motor.enableVoltageCompensation(12);

<<<<<<< HEAD
        motor.getEncoder().setPositionConversionFactor(Units.degreesToRadians(360.0) / Constants.PIVOT_GEAR_RATIO); // In
                                                                                                                    // Degrees

        motor.getEncoder()
                .setVelocityConversionFactor((Units.degreesToRadians(360.0) / Constants.PIVOT_GEAR_RATIO) / 60.0); // Degrees
                                                                                                                   // Per
                                                                                                                   // Second
=======
    motor
        .getEncoder()
        .setPositionConversionFactor(
            Units.degreesToRadians(360.0) / Constants.PIVOT_GEAR_RATIO); // In Degrees
>>>>>>> af71b99b7a39e8885987bfc594d4a9725aa38e49

    motor
        .getEncoder()
        .setVelocityConversionFactor(
            (Units.degreesToRadians(360.0) / Constants.PIVOT_GEAR_RATIO)
                / 60.0); // Degrees Per Second

    setGoal(Constants.ARM_OFF_SET_RADS);
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    double feedforward = armFeedforward.calculate(setpoint.position, setpoint.velocity);
    motor.setVoltage(output + feedforward);
  }

<<<<<<< HEAD
    public void burnFlash() {
        motor.burnFlash();
    }
=======
  @Override
  protected double getMeasurement() {
    return motor.getEncoder().getPosition() + Constants.ARM_OFF_SET_RADS;
  }
>>>>>>> af71b99b7a39e8885987bfc594d4a9725aa38e49
}
