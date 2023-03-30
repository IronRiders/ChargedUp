package frc.robot.subsystems;

import java.util.Random;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightsSubsystem extends SubsystemBase {

  public enum LightPattern {
    CONE,
    CUBE,
    GREEN,
    YELLOW,
    RAINBOW,
    CHARGING_STATION,
    NOISE
  }

  private static LightPattern lightPattern =
      LightPattern.CHARGING_STATION; // Defaults to team colors

  AddressableLED addressableLed = new AddressableLED(Constants.LED_STRIP_PORT);
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LED_STRIP_BUFFER_SIZE);
  SendableChooser<String> lightPatternChooser = new SendableChooser<>();
  String lastSelectedLightPattern;
  int rainbowFirstLedHue = 0;

  public LightsSubsystem() {
    addressableLed.setLength(ledBuffer.getLength());

    lightPatternChooser.addOption("Cone", "CONE");
    lightPatternChooser.addOption("Cube", "CUBE");
    lightPatternChooser.addOption("Green", "GREEN");
    lightPatternChooser.addOption("Yellow", "YELLOW");
    lightPatternChooser.addOption("Rainbow", "RAINBOW");
    lightPatternChooser.addOption("Charging Station", "CHARGING_STATION");
    lightPatternChooser.addOption("Noise", "NOISE");

    SmartDashboard.putData("Light Patterns", lightPatternChooser);
  }

  @Override
  public void periodic() {
    // Runs only when value is changed so that other things can make it change freely
    if (!lightPatternChooser.getSelected().equals(lastSelectedLightPattern)) {
      setLightPattern(LightPattern.valueOf(lightPatternChooser.getSelected()));
      lastSelectedLightPattern = lightPatternChooser.getSelected();
    }

    switch (lightPattern) {
      case CONE:
      case YELLOW:
        setColorRGB(255, 255, 0);
        break;
      case CUBE:
        setColorRGB(255, 0, 255);
        break;
      case GREEN:
        setColorRGB(0, 200, 0);
        break;
      case RAINBOW:
        rainbow();
        break;
      case CHARGING_STATION:
        chargingStation();
        break;
      case NOISE:
        noise();
        break;
    }
  }

  private void chargingStation() {
    WPI_Pigeon2 pigeon = new WPI_Pigeon2(15);
    if (pigeon.getPitch() < 2.5 && pigeon.getPitch() > -2.5) {
      pigeon.close();
      if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
        setColorRGB(255, 0, 0);
      } else {
        setColorRGB(0, 0, 255);
      }
      addressableLed.setData(ledBuffer);
      addressableLed.start();
    } else {
      setColorRGB(0, 0, 0);
    }
  }

  private void rainbow() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      final int hue = (rainbowFirstLedHue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    rainbowFirstLedHue += 1;
    rainbowFirstLedHue %= 180;
    addressableLed.setData(ledBuffer);
    addressableLed.start();
  }

  private void noise() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(
          i, new Random().nextInt(256), new Random().nextInt(256), new Random().nextInt(256));
    }
    addressableLed.setData(ledBuffer);
    addressableLed.start();
  }

  public static void setColorGrabObject(GrabObject object) {
    if (object == GrabObject.CONE) {
      setLightPattern(LightPattern.CONE);
    } else {
      setLightPattern(LightPattern.CUBE);
    }
  }

  public void setColorRGB(int r, int g, int b) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, r, b, g); // BUG ON WPILIB: MIXES BLUE AND GREEN
    }
    addressableLed.setData(ledBuffer);
    addressableLed.start();
  }

  public void setColorHSV(int h, int s, int v) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, h, s, v);
    }
    addressableLed.setData(ledBuffer);
    addressableLed.start();
  }

  public static void setLightPattern(LightPattern newLightPattern) {
    lightPattern = newLightPattern;
  }
}
