package frc.robot;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
  // Manipulator
  public static final int MANIPULATOR_PORT1 = 9;
  public static final int MANIPULATOR_PORT2 = 10;
  public static final double MANIPULATOR_POWER = 0.8;
  public static final int MANIPULATOR_CURRENT_LIMIT = 10;
  public static final int MANIPULATOR_CURRENT_LIMIT_CONE = 15;
  public static final int MANIPULATOR_CURRENT_LIMIT_BOX = 20;
  public static final double MANIPULATOR_PID_KP = 0.0;
  public static final double MANIPULATOR_PID_KI = 0.0;
  public static final double MANIPULATOR_PID_KD = 0.0;
  public static final double MANIPULATOR_PID_TOLERANCE = 0.5;

  // Arm
  public static final double BOX_CLIMBER_MOTOR_POWER = 1.0;
  public static final int BOX_CLIMBER_MOTOR_CURRENT_LIMIT = 10;
  public static final double ARM_MOTOR_POWER = 1.0;
  public static final int ARM_MOTOR_CURRENT_LIMIT = 10;
  public static final double ARM_EXTEND_RETRACT_PID_KP = 0.0;
  public static final double ARM_EXTEND_RETRACT_PID_KI = 0.0;
  public static final double ARM_EXTEND_RETRACT_PID_KD = 0.0;
  public static final double ARM_EXTEND_RETRACT_PID_TOLERANCE = 0.3;
  public static final double ARM_RAISE_LOWER_PID_KP = 0.0;
  public static final double ARM_RAISE_LOWER_PID_KI = 0.0;
  public static final double ARM_RAISE_LOWER_PID_KD = 0.0;
  public static final double ARM_RAISE_LOWER_PID_TOLERANCE = 0.3;

  // Joystick
  public static final double DEADBAND = 0.06;
  public static final double EXPONENT = 0.1; // between 0 and 1

  // Drivetrain
  public static final double DRIVE_SPEED_MULT = 1.0;
  public static final double DRIVE_SPEED_AUTO = 0.1 / DRIVE_SPEED_MULT;
  public static final int DRIVE_CURRENT_LIMIT = 40;
  public static final double MOVEMENT_SPEED = 1; // meters per second
  public static final double TURN_SPEED = 1; // radians per second
  public static final double DIAMETER = 7.9; // Inches
  public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(DIAMETER * Math.PI);
  public static final double AUTO_WHEELPID_KP = 0.2;
  public static final double GEARING = 4;

  // Ports
  public static final int WHEEL_PORT_FRONT_LEFT = 1;
  public static final int WHEEL_PORT_REAR_LEFT = 3;
  public static final int WHEEL_PORT_FRONT_RIGHT = 2;
  public static final int WHEEL_PORT_REAR_RIGHT = 4;
  public static final int ARM_BOX_CLIMBER_PORT = 5;
  public static final int ARM_RAISE_LOWER_PORT = 6;

  // Straighten Robot
  public static final double STRAIGHTEN_TALORANCE_ANGLE = 5;
  public static final double STRAIGHTEN_ROBOT_TURN_SPEED = 1;

  //Auto
  public static final double AUTO_POSITION_KP = 0;
  public static final double AUTO_THETACONTROLLER_KP = 9;
  public static final double AUTO_XCONTROLLER_KP = 0.45;
  public static final double AUTO_YCONTROLLER_KP = 0.06;
  public static final double DRIVE_ACCELERATION_AUTO = 1;
  public static final double Gearing = 4;
  public static final PathConstraints SlowAutoConstraints = new PathConstraints(0.4, 0.4);
  public static final PathConstraints MediumAutoConstraints = new PathConstraints(2, 2);
  public static final PathConstraints FastAutoConstraints = new PathConstraints(3, 3);
  public static final PathConstraints TooFastAutoConstraints = new PathConstraints(4, 4);
  public static final double ANGLETOLERANCE = 1;

  //Grid Poses
  public static final Pose2d STATION1 = new Pose2d(new Translation2d(1.86, 3.28), Rotation2d.fromDegrees(180));
}
