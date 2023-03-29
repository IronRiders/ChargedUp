package frc.robot.subsystems;

import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightsSubsystem extends SubsystemBase {

  public enum LightPattern {
    CONE,
    CUBE,
    GREEN,
    YELLOW,
    RAINBOW,
    NOISE
  }

  private static LightPattern lightPattern = LightPattern.CUBE; // Defaults to team colors

  AddressableLED addressableLed = new AddressableLED(Constants.LED_STRIP_PORT);
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LED_STRIP_BUFFER_SIZE);
  int ranbowFirstLedHue = 0;

  public LightsSubsystem() {
    addressableLed.setLength(ledBuffer.getLength());
  }

  @Override
  public void periodic() {
    switch (lightPattern) {
      case CONE:
        setColorRGB(255, 255, 0);
        break;
      case CUBE:
        setColorRGB(255, 0, 255);
        break;
      case GREEN:
        setColorRGB(0, 200, 0);
        break;
      case YELLOW:
        setColorRGB(255, 255, 0);
        break;
      case RAINBOW:
        rainbow();
        break;
      case NOISE:
        noise();
        break;
    }
  }

  public void rainbow() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      final int hue = (ranbowFirstLedHue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    ranbowFirstLedHue += 1;
    ranbowFirstLedHue %= 180;
    addressableLed.setData(ledBuffer);
    addressableLed.start();
  }

  public void noise() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(
          i, new Random().nextInt(256), new Random().nextInt(256), new Random().nextInt(256));
    }
    addressableLed.setData(ledBuffer);
    addressableLed.start();
  }

  public void setColorGrabObject(GrabObject object) {
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

  public void setLightPattern(LightPattern newLightPattern) {
    lightPattern = newLightPattern;
  }
}
