package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import frc.robot.Constants;

public class DifferentialDrive extends SubsystemBase {
  // creates variables & motors
  private boolean inverted;
  private CANSparkMax frontLeftMotor;
  private CANSparkMax backLeftMotor;
  private CANSparkMax frontRightMotor;
  private CANSparkMax backRightMotor;

  private final DifferentialDriveKinematics kinematics;

  public DifferentialDrive() {
    // Assigns motors to variables
    frontLeftMotor = new CANSparkMax(Constants.WHEEL_PORT_FRONT_LEFT, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(Constants.WHEEL_PORT_FRONT_RIGHT, MotorType.kBrushless);
    backLeftMotor = new CANSparkMax(Constants.WHEEL_PORT_REAR_LEFT, MotorType.kBrushless);
    backRightMotor = new CANSparkMax(Constants.WHEEL_PORT_REAR_LEFT, MotorType.kBrushless);

    // sets inverted for motors
    frontLeftMotor.setInverted(true);
    frontRightMotor.setInverted(false);
    backLeftMotor.setInverted(true);
    backRightMotor.setInverted(false);
    inverted = false;

    // Set all motor current limit
    frontLeftMotor.setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
    frontRightMotor.setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
    backLeftMotor.setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
    backRightMotor.setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);

    // Set all motors to break
    frontLeftMotor.setIdleMode(IdleMode.kBrake);
    frontRightMotor.setIdleMode(IdleMode.kBrake);
    backLeftMotor.setIdleMode(IdleMode.kBrake);
    backRightMotor.setIdleMode(IdleMode.kBrake);

    // creating diferential drive kinematics with track width of 0.5715 m
    kinematics = new DifferentialDriveKinematics(0.5715);
  }

  public void invertDrive() {
    inverted = !inverted;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();

    SmartDashboard.putBoolean("isInverted", inverted);
  }

  // Use strafe parameter just to be backwards compatible

  public void updateSpeed(double strafe, double drive, double turn, boolean useInverted) {
    double xSpeed = drive * Constants.MOVEMENT_SPEED;
    if (useInverted && inverted) {
      xSpeed = -xSpeed;
    }

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, 0, turn * -Constants.TURN_SPEED);

    // Convert to wheel speeds
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

    // Left velocity
    double leftVelocity = wheelSpeeds.leftMetersPerSecond;

    // Right velocity
    double rightVelocity = wheelSpeeds.rightMetersPerSecond;
    // Motor control groups
    MotorControllerGroup leftMotors = new MotorControllerGroup(backLeftMotor, frontLeftMotor);

    MotorControllerGroup rightMotors = new MotorControllerGroup(backRightMotor, frontRightMotor);
    // Set velocity
    leftMotors.set(leftVelocity);
    rightMotors.set(rightVelocity);
  }

  public void stop() {
    updateSpeed(0, 0, 0, false);
  }
}
