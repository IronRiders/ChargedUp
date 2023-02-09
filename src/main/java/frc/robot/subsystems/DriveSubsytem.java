package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsytem extends SubsystemBase {

  private boolean inverted;
  private ChassisSpeeds actualChassisSpeeds;
  private ChassisSpeeds targetChassisSpeeds;
  private MecanumWheel[] motors = new MecanumWheel[4];
  private final MecanumDriveKinematics kinematics;
  public final WPI_Pigeon2 pigeon;
  public final Field2d field;
  private final MecanumDrivePoseEstimator poseEstimator;
  private final MecanumDriveWheelPositions wheelPositions;

  public DriveSubsytem() {
    motors[0] = new MecanumWheel(Constants.WHEEL_PORT_FRONT_LEFT, true);
    motors[1] = new MecanumWheel(Constants.WHEEL_PORT_FRONT_RIGHT, false);
    motors[2] = new MecanumWheel(Constants.WHEEL_PORT_REAR_LEFT, true);
    motors[3] = new MecanumWheel(Constants.WHEEL_PORT_REAR_RIGHT, false);
    inverted = false;

    // meter per second
    kinematics =
        new MecanumDriveKinematics(
            new Translation2d(0.28575, 0.2267),
            new Translation2d(0.28575, -0.2267),
            new Translation2d(-0.28575, 0.2267),
            new Translation2d(-0.28575, -0.2267));

    pigeon = new WPI_Pigeon2(15);
    targetChassisSpeeds = new ChassisSpeeds();
    field = new Field2d();
    wheelPositions =
        new MecanumDriveWheelPositions(
            motors[0].getWheelPostions(),
            motors[1].getWheelPostions(),
            motors[2].getWheelPostions(),
            motors[3].getWheelPostions());
    poseEstimator =
        new MecanumDrivePoseEstimator(
            getKinematics(), pigeon.getRotation2d(), wheelPositions, new Pose2d());
  }

  public void invertDrive() {
    inverted = !inverted;
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        motors[0].getVelocity(),
        motors[1].getVelocity(),
        motors[2].getVelocity(),
        motors[3].getVelocity());
  }

  public void periodic() {
    // Update pose estimator with drivetrain sensors
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), pigeon.getRotation2d(), wheelPositions);
  }

  public Pose2d getPose2d() {
    return poseEstimator.getEstimatedPosition();
  }

  public MecanumDriveKinematics getKinematics() {
    return kinematics;
  }

  public void SetWheelSpeeds(MecanumDriveWheelSpeeds speed, boolean needPID) {
    motors[0].setVelocity(speed.frontLeftMetersPerSecond, needPID);
    motors[1].setVelocity(speed.frontRightMetersPerSecond, needPID);
    motors[2].setVelocity(speed.rearLeftMetersPerSecond, needPID);
    motors[3].setVelocity(speed.rearRightMetersPerSecond, needPID);
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean needPID) {
    targetChassisSpeeds = chassisSpeeds;
    SetWheelSpeeds(kinematics.toWheelSpeeds(chassisSpeeds), needPID);
  }

  public void setChassisSpeeds(double strafe, double drive, double turn, boolean needPID) {

    double xSpeed = drive * MecanumWheel.getMaxLinearVelocity();
    double ySpeed = strafe * MecanumWheel.getMaxLinearVelocity();
    double turnSpeed = turn * -getMaxRotationalVelocity();

    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, pigeon.getRotation2d());
    setChassisSpeeds(chassisSpeeds, needPID);
  }

  public double getMaxRotationalVelocity() {
    return Math.abs(
        (kinematics.toChassisSpeeds(
                new MecanumDriveWheelSpeeds(
                    MecanumWheel.getMaxLinearVelocity(),
                    -MecanumWheel.getMaxLinearVelocity(),
                    MecanumWheel.getMaxLinearVelocity(),
                    -MecanumWheel.getMaxLinearVelocity())))
            .omegaRadiansPerSecond);
  }

  public void stop() {
    setChassisSpeeds(0, 0, 0, true);
  }

  public void resetOdometry(Pose2d pose2d) {
    poseEstimator.resetPosition(pigeon.getRotation2d(), wheelPositions, pose2d);
  }

  public MecanumDrivePoseEstimator getPoseEstimator() {
    return poseEstimator;
  }
}
