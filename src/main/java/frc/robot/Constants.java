package frc.robot;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  // Joystick
  public static final double DEADBAND = 0.07;
  public static final double EXPONENT = 0.1; // between 0 and 1

  // Drivetrain
  public static final double DRIVE_SPEED_MULT = 1.0;
  public static final double DRIVE_SPEED_AUTO = 0.1 / DRIVE_SPEED_MULT;
  public static final int DRIVE_CURRENT_LIMIT = 40;
  public static final double MOVEMENT_SPEED = 1; // meters per second
  public static final double TURN_SPEED = 1; // radians per second
  public static final double DIAMETER = 8; // Inches
  public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(DIAMETER * Math.PI);
  public static final double AUTO_WHEELPID_KP = 0;
  public static final double GEARING = 11.314285714;

  // Ports
  public static final int WHEEL_PORT_FRONT_LEFT = 2;
  public static final int WHEEL_PORT_REAR_LEFT = 1;
  public static final int WHEEL_PORT_FRONT_RIGHT = 3;
  public static final int WHEEL_PORT_REAR_RIGHT = 4;
  public static final int ARM_CLIMBER_PORT = 6;
  public static final int PIVOT_PORT = 5;
  public static final int MANIPULATOR_PORT1 = 7;
  public static final int MANIPULATOR_PORT2 = 8;

  // Straighten Robot
  public static final double STRAIGHTEN_TALORANCE_ANGLE = 5;
  public static final double STRAIGHTEN_ROBOT_TURN_SPEED = 0.1;

  // Auto
  public static final double AUTO_POSITION_KP = 0;
  public static final double AUTO_THETACONTROLLER_KP = 0;
  public static final double AUTO_XCONTROLLER_KP = 0;
  public static final double AUTO_YCONTROLLER_KP = 0;
  public static final double DRIVE_ACCELERATION_AUTO = 1;
  public static final PathConstraints SlowAutoConstraints = new PathConstraints(1, 1);
  public static final PathConstraints MediumAutoConstraints = new PathConstraints(3, 3);
  public static final PathConstraints FastAutoConstraints = new PathConstraints(4, 4);
  public static final PathConstraints TooFastAutoConstraints = new PathConstraints(5, 5);
  public static final double ANGLETOLERANCE = 1;

  // Auto Leveling
  public static final double FORWARD_VELOCITY = 0.065;
  public static final double STRAFE_VELOCITY = .01;
  public static final double ANGLE_TOLERANCE = 1.3;

  // Lights
  public static final int LED_STRIP_PORT = 0;
  public static final int LED_STRIP_BUFFER_SIZE = 18; // 30 lights per meter

  // Vision Stuff
  public static final Transform3d RobotToCam =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(4), -Units.inchesToMeters(7), Units.inchesToMeters(32.75)),
          new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)));
    public static final Transform3d RobotTopiCam =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(8.932), -Units.inchesToMeters(14), Units.inchesToMeters(30.5)),
              new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)));

  // Arm
  public static final int ARM_CURRENT_LIMIT = 10;
  public static final double Arm_POWER = 0.9;
  public static final double ARM_KP = 0.6;
  public static final double Arm_GEAR_RATIO = 48;
  public static final TrapezoidProfile.Constraints kConstraints =
      new TrapezoidProfile.Constraints(75, 120);

  // Pivot Stuff
  public static final int PIVOT_CURRENT_LIMIT = 12;
  public static final double Pivot_KP = 24;
  public static final double Pivot_KI = 0;
  public static final double Pivot_KD = 0;
  public static final double SHOULDER_VELOCITY_DEG = 70;
  public static final double SHOULDER_ACCELERATION_DEG = 250;
  public static final double ARM_OFF_SET_RADS = Units.degreesToRadians(20);
  public static final double PIVOT_GEAR_RATIO = 528.57;
  public static final int PIVOT_MOTOR_CURRENT_LIMIT = 15;

  // Angles
  public static final double L3ANGLE = 118;
  public static final double L2ANGLE = 103;
  public static final double L1ANGLE = 65;
  public static final double LGROUND = 45;
  public static final double LHUMAN = 105;

  // Feedforward for Pivot
  public static final double PIVOT_KS = 0;
  public static final double PIVOT_KGR = 0;
  public static final double PIVOT_KV = 0;
  public static final double PIVOT_KA = 0;

  // Manipulator
  public static final int MANIPULATOR_CURRENT_LIMIT = 6;
  public static final double MANIPULATOR_SPEED_CONE = 0.3;
  public static final double MANIPULATOR_SPEED_BOX = 0.2;
  public static final double MANIPULATOR_PID_KP = 0.0;
  public static final double MANIPULATOR_PID_KI = 0.0;
  public static final double MANIPULATOR_PID_KD = 0.0;
  public static final double MANIPULATOR_PID_TOLERANCE = 0.5;
  public static final double MANIPULATOR_SETPOINT = 0.0;
  public static final double STALL_CURRENT = 1.2;
  public static final double STALL_SPEED = 0.0005;
}
