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
    RAINBOW,
    GREEN_YELLOW,
    CONE_CUBE,
    BLACK_WHITE,
    NOISE
  }

  private static LightPattern lightPattern = LightPattern.GREEN_YELLOW; // Defaults to team colors

  AddressableLED addressableLed = new AddressableLED(Constants.LED_STRIP_PORT);
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LED_STRIP_BUFFER_SIZE);
  int ranbowFirstLedHue = 0;
  int checkerboardOffset = 0;

  public LightsSubsystem() {
    // addressableLed.setLength(ledBuffer.getLength());
    // addressableLed.setData(ledBuffer);
    // addressableLed.start();
    // I don't think we need this
  }

  @Override
  public void periodic() {
    switch (lightPattern) {
      case CONE: 
        setColorRGB(255, 0, 255); break;
      case CUBE: 
        setColorRGB(255, 255, 0); break;
      case RAINBOW: 
        rainbow(); break;
      case GREEN_YELLOW: 
        checkerboard(0, 255, 0, 255, 0, 255, 6, 1); break;
      case CONE_CUBE:
        checkerboard(255, 0, 255, 255, 255, 0, 6, 1); break;
      case BLACK_WHITE: 
        checkerboard(0, 0, 0, 255, 255, 255, 6, 1); break;
      case NOISE: 
        noise(); break;
    }
  }

  public void rainbow() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      final int hue = (ranbowFirstLedHue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    ranbowFirstLedHue += 3;
    ranbowFirstLedHue %= 180;
    addressableLed.setData(ledBuffer);
    addressableLed.start();
  }

  public void checkerboard(int r1, int g1, int b1, int r2, int g2, int b2, int blocks, int speed) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      boolean isColorOne =
        ((i + checkerboardOffset + (ledBuffer.getLength() / 4))
              / (Double.valueOf(ledBuffer.getLength()) / Double.valueOf(blocks - 1)))
            % 2
          == 0;
      int r = (isColorOne ? r1 : r2);
      int g = (isColorOne ? g1 : g2);
      int b = (isColorOne ? b1 : b2);
      ledBuffer.setRGB(i, r, g, b);
    }
    checkerboardOffset += speed;
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
      lightPattern = LightPattern.CONE;
    } else {
      lightPattern = LightPattern.CUBE;
    }
  }

  public void setColorRGB(int r, int g, int b) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, r, b, g); // BUG ON WPILIB: MIXES BLUE AND GREEN
    }
    addressableLed.setData(ledBuffer);
  }

  public void setColorHSV(int h, int s, int v) {

    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, h, s, v);
    }
    addressableLed.setData(ledBuffer);
  }

  public void setLightPattern(LightPattern newLightPattern) {
    lightPattern = newLightPattern;
  }
}
