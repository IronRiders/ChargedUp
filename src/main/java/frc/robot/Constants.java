package frc.robot;

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
  public static final double MANIPULATOR_SETPOINT = 0.0;

  // Arm
  public static final double BOX_CLIMBER_MOTOR_POWER = 1.0;
  public static final int BOX_CLIMBER_MOTOR_CURRENT_LIMIT = 10;
  public static final double ARM_MOTOR_POWER = 1.0;
  public static final int ARM_MOTOR_CURRENT_LIMIT = 10;
  public static final double ARM_EXTEND_RETRACT_PID_KP = 0.0;
  public static final double ARM_EXTEND_RETRACT_PID_KI = 0.0;
  public static final double ARM_EXTEND_RETRACT_PID_KD = 0.0;
  public static final double ARM_EXTEND_RETRACT_PID_TOLERANCE = 0.3;
  public static final double ARM_EXTEND_RETRACT_SETPOINT = 0.0;
  public static final double ARM_RAISE_LOWER_PID_KP = 0.0;
  public static final double ARM_RAISE_LOWER_PID_KI = 0.0;
  public static final double ARM_RAISE_LOWER_PID_KD = 0.0;
  public static final double ARM_RAISE_LOWER_PID_TOLERANCE = 0.3;
  public static final double ARM_RAISE_LOWER_SETPOINT = 0.0;

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

  // Auto Leveling
  public static final double FORWARD_VELOCITY = .05;
  public static final double STRAFE_VELOCITY = .03;
  public static final double ANGLE_TOLERANCE = 1.5;
}
