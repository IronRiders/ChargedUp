package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightsSubsystem extends SubsystemBase {

    AddressableLED addressableLed = new AddressableLED(Constants.LED_STRIP_PORT);
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LED_STRIP_BUFFER_SIZE);
    int ranbowFirstLedHue = 0;
    int offset = 0;
    
    public LightsSubsystem() {      
      addressableLed.setLength(ledBuffer.getLength());
    }

    public void periodic() {
      checkerboard(255, 255, 255, 0, 0, 0, 30, offset);
      offset += 1;
      
    }

    public void rainbow() {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        final int hue = (ranbowFirstLedHue + (i * 180 / ledBuffer.getLength())) % 180;
        ledBuffer.setHSV(i, hue, 255, 128);
      }
      ranbowFirstLedHue += 3;
      ranbowFirstLedHue %= 180;
    }

    public void checkerboard(int r1, int g1, int b1, int r2, int g2, int b2, int blocks, int offset) {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        boolean isColorOne = ((i + offset + (ledBuffer.getLength() / 4))/(Double.valueOf(ledBuffer.getLength())/Double.valueOf(blocks - 1))) % 2 == 0;
        int r = (isColorOne ? r1 : r2);
        int g = (isColorOne ? g1 : g2);
        int b = (isColorOne ? b1 : b2);
        ledBuffer.setRGB(i, r, g, b);
      }
    } 
    
    public void setColorRGB(int r, int g, int b) {
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, r, g, b);
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
}
