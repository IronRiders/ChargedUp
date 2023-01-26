package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MecanumDrive extends SubsystemBase {
  private boolean inverted;
  private CANSparkMax[] motors;

  private final MecanumDriveKinematics kinematics;

  public MecanumDrive() {
    this.motors = new CANSparkMax[4];
    this.motors[0] = new CANSparkMax(Constants.WHEEL_PORT_FRONT_LEFT, MotorType.kBrushless);
    this.motors[1] = new CANSparkMax(Constants.WHEEL_PORT_FRONT_RIGHT, MotorType.kBrushless);
    this.motors[2] = new CANSparkMax(Constants.WHEEL_PORT_REAR_LEFT, MotorType.kBrushless);

    this.motors[0].setInverted(true);
    this.motors[1].setInverted(false);
    this.motors[2].setInverted(true);
    this.motors[3].setInverted(false);
    inverted = false;

    motors[0].setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
    motors[1].setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
    motors[2].setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
    motors[3].setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);

    motors[0].setIdleMode(IdleMode.kBrake);
    motors[1].setIdleMode(IdleMode.kBrake);
    motors[2].setIdleMode(IdleMode.kBrake);
    motors[3].setIdleMode(IdleMode.kBrake);

    // meter per second
    kinematics =
        new MecanumDriveKinematics(
            new Translation2d(0.28575, 0.2267),
            new Translation2d(0.28575, -0.2267),
            new Translation2d(-0.28575, 0.2267),
            new Translation2d(-0.28575, -0.2267));
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

  public void updateSpeed(double strafe, double drive, double turn, boolean useInverted) {
    double xSpeed = drive * Constants.MOVEMENT_SPEED;
    double ySpeed = strafe * Constants.MOVEMENT_SPEED;
    if (useInverted && inverted) {
      xSpeed = -xSpeed;
      ySpeed = -ySpeed;
    }

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turn * -Constants.TURN_SPEED);
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

    this.motors[0].set(
        wheelSpeeds.frontLeftMetersPerSecond
            * Constants.DRIVE_SPEED_MULT
            / Constants.MOVEMENT_SPEED);
    this.motors[1].set(
        wheelSpeeds.frontRightMetersPerSecond
            * Constants.DRIVE_SPEED_MULT
            / Constants.MOVEMENT_SPEED);
    this.motors[2].set(
        wheelSpeeds.rearLeftMetersPerSecond
            * Constants.DRIVE_SPEED_MULT
            / Constants.MOVEMENT_SPEED);
    this.motors[3].set(
        wheelSpeeds.rearRightMetersPerSecond
            * Constants.DRIVE_SPEED_MULT
            / Constants.MOVEMENT_SPEED);
  }

  public void stop() {
    updateSpeed(0, 0, 0, false);
  }
}
