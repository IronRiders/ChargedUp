package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Vision;

public class DriveSubsystem extends SubsystemBase {

  private boolean inverted;
  private ChassisSpeeds ActualChassisSpeeds;
  private ChassisSpeeds targetChassisSpeeds;
  private MecanumWheel frontLeftMotor;
  private MecanumWheel frontRightMotor;
  private MecanumWheel rearRightMotor;
  private MecanumWheel rearLeftMotor;
  private final MecanumDriveKinematics kinematics;
  public final WPI_Pigeon2 pigeon;
  public final Field2d field;
  private final MecanumDrivePoseEstimator poseEstimator;
  private final Vision vision = new Vision();
  private static ProfiledPIDController profiledThetaController =
      new ProfiledPIDController(
          0.4,
          0,
          0,
          new TrapezoidProfile.Constraints(
              Units.rotationsToRadians(0.75), Units.rotationsToRadians(1.5)));
  private static PIDController thetaController =
      new PIDController(
          profiledThetaController.getP(),
          profiledThetaController.getI(),
          profiledThetaController.getD());
  private static PIDController xController = new PIDController(0.15, 0, 0);
  private static PIDController yController = new PIDController(0.15, 0, 0);

  public DriveSubsystem() {
    frontLeftMotor = new MecanumWheel(Constants.WHEEL_PORT_FRONT_LEFT, true);
    frontRightMotor = new MecanumWheel(Constants.WHEEL_PORT_FRONT_RIGHT, false);
    rearRightMotor = new MecanumWheel(Constants.WHEEL_PORT_REAR_LEFT, true);
    rearLeftMotor = new MecanumWheel(Constants.WHEEL_PORT_REAR_RIGHT, false);
    inverted = false;

    // meter per second
    kinematics =
        new MecanumDriveKinematics(
            new Translation2d(0.2413, 0.28893008),
            new Translation2d(0.2413, -0.28893008),
            new Translation2d(-0.2413, 0.28893008),
            new Translation2d(-0.2413, -0.28893008));

    pigeon = new WPI_Pigeon2(15);
    targetChassisSpeeds = new ChassisSpeeds();
    field = new Field2d();
    poseEstimator =
        new MecanumDrivePoseEstimator(
            getKinematics(), pigeon.getRotation2d(), getWheelPositions(), new Pose2d());
  }

  public void invertDrive() {
    inverted = !inverted;
  }

  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(
        frontLeftMotor.getWheelPostions(),
        frontRightMotor.getWheelPostions(),
        rearLeftMotor.getWheelPostions(),
        rearRightMotor.getWheelPostions());
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        frontLeftMotor.getVelocity(),
        frontRightMotor.getVelocity(),
        rearRightMotor.getVelocity(),
        rearLeftMotor.getVelocity());
  }

  public void periodic() {
    // Update pose estimator with drivetrain sensors
    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(), pigeon.getRotation2d(), getWheelPositions());
    SmartDashboard.putNumber("auto yaw", pigeon.getYaw());
    vision
        .getEstimatedGlobalPose(getPose2d())
        .ifPresent(
            pose -> {
              getPoseEstimator()
                  .addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
            });

    // Simple Simulation
    field.setRobotPose(getPose2d());
    field
        .getObject("Target")
        .setPose(
            new Pose2d(
                xController.getSetpoint(),
                yController.getSetpoint(),
                new Rotation2d(getThetaController().getSetpoint())));

    // Tuning
    NetworkTableInstance.getDefault().flush();

    SmartDashboard.putNumber("x controller", getPose2d().getX());
    SmartDashboard.putNumber("x Controller (target)", xController.getSetpoint());
    SmartDashboard.putNumber("Y controller", getPose2d().getY());
    SmartDashboard.putNumber("y Controller (target)", yController.getSetpoint());
    SmartDashboard.putNumber("Theta controller (Degrees)", getPose2d().getRotation().getDegrees());
    SmartDashboard.putNumber(
        "Theta setPoint (Target))", Math.toDegrees(profiledThetaController.getSetpoint().position));

    ActualChassisSpeeds = kinematics.toChassisSpeeds(getWheelSpeeds());

    SmartDashboard.putNumber("Drive/VX", ActualChassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Drive/VY", ActualChassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber(
        "Drive/Omega Degrees", Units.radiansToDegrees(ActualChassisSpeeds.omegaRadiansPerSecond));

    SmartDashboard.putNumber("Drive/Target VX", targetChassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Drive/Target VY", targetChassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber(
        "Drive/Target Omega Degrees",
        Units.radiansToDegrees(targetChassisSpeeds.omegaRadiansPerSecond));
    SmartDashboard.putNumber("Gyro Angle", pigeon.getAngle());

    SmartDashboard.putNumber("Pitch", pigeon.getPitch());
  }

  public Pose2d getPose2d() {
    return poseEstimator.getEstimatedPosition();
  }

  public MecanumDriveKinematics getKinematics() {
    return kinematics;
  }

  public void setWheelSpeeds(MecanumDriveWheelSpeeds speed, boolean needPID) {
    frontLeftMotor.setVelocity(speed.frontLeftMetersPerSecond, needPID);
    frontRightMotor.setVelocity(speed.frontRightMetersPerSecond, needPID);
    rearRightMotor.setVelocity(speed.rearLeftMetersPerSecond, needPID);
    rearLeftMotor.setVelocity(speed.rearRightMetersPerSecond, needPID);
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean needPID) {
    targetChassisSpeeds = chassisSpeeds;
    setWheelSpeeds(kinematics.toWheelSpeeds(targetChassisSpeeds), needPID);
  }

  public void setChassisSpeeds(double strafe, double drive, double turn, boolean needPID) {

    double xSpeed = drive * MecanumWheel.getMaxLinearVelocity();
    double ySpeed = strafe * MecanumWheel.getMaxLinearVelocity();
    double turnSpeed = turn * -getMaxRotationalVelocity();

    ChassisSpeeds chassisSpeeds;

    if (!needPID)
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, pigeon.getRotation2d());
    else chassisSpeeds = new ChassisSpeeds(-xSpeed, -ySpeed, -turnSpeed);

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
    poseEstimator.resetPosition(pigeon.getRotation2d(), getWheelPositions(), pose2d);
  }

  public MecanumDrivePoseEstimator getPoseEstimator() {
    return poseEstimator;
  }

  public ProfiledPIDController getProfiledThetaController() {
    return profiledThetaController;
  }

  public PIDController getThetaController() {
    return thetaController;
  }

  public PIDController getxController() {
    return xController;
  }

  public PIDController getyController() {
    return yController;
  }

  public ChassisSpeeds getChasssisSpeeds() {
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public void burnFlash() {
    frontLeftMotor.burnFlash();
    frontRightMotor.burnFlash();
    rearRightMotor.burnFlash();
    rearLeftMotor.burnFlash();
  }
}
