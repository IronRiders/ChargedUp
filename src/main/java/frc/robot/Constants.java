package frc.robot;

public final class Constants {
  public static final int MANIPULATOR_PORT1 = 9;
  public static final int MANIPULATOR_PORT2 = 10;
  public static final double MANIPULATOR_POWER = 0.8;
  public static final int MANIPULATOR_CURRENT_LIMIT = 10;
  public static final int MANIPULATOR_CURRENT_LIMIT_CONE = 15;
  public static final int MANIPULATOR_CURRENT_LIMIT_BOX = 20;

  // Joystick
  public static final double DEADBAND = 0.06;
  public static final double EXPONENT = 0.1; // between 0 and 1

  // Drivetrain
  public static final double DRIVE_SPEED_MULT = 1.0; // should be between 0 and 1
  public static final double DRIVE_SPEED_AUTO = 0.1 / DRIVE_SPEED_MULT;
  public static final int DRIVE_CURRENT_LIMIT = 40;
  public static final double MOVEMENT_SPEED = 1; // meters per second
  public static final double TURN_SPEED = 1; // radians per second

  // Ports
  public static final int WHEEL_PORT_FRONT_LEFT = 1;
  public static final int WHEEL_PORT_REAR_LEFT = 3;
  public static final int WHEEL_PORT_FRONT_RIGHT = 2;
  public static final int WHEEL_PORT_REAR_RIGHT = 4;
}
