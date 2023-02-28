package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import frc.robot.Constants;

public class UltrasonicSensorSubsystem {
  private Ultrasonic ultrasonicSensor;

  public UltrasonicSensorSubsystem() {
    ultrasonicSensor =
        new Ultrasonic(Constants.ULTRASONIC_SENSOR_PORT1, Constants.ULTRASONIC_SENSOR_PORT2);
  }

  public boolean isObjectNearby() {
    return ultrasonicSensor.getRangeMM() / 10 <= Constants.DISTANCE_TO_OBJECT;
  }
}
