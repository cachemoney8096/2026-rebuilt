package frc.robot.subsystems.lights;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import frc.robot.RobotMap;

import java.util.TreeMap;

public class Lights {
  /**
   * Tree map of LightCode enums and integers that represent the R, G, B values of each light code
   * enum
   */
  private TreeMap<LightCode, RGBWColor> lightOptionsMap;

  private CANdle candle = new CANdle(RobotMap.CANDLE_CAN_ID, RobotMap.RIO_CAN_BUS);
  private CANdleConfiguration config = new CANdleConfiguration();
  private LightCode currentLightStatus = LightCode.OFF;

  public enum LightCode {
    OFF, // BLACK
    DISABLED, // ORANGE
    INTAKING, // BLINK RED
    AUTO_LOCKED, // GREEN
    READY_TO_CLIMB, // RAINBOW
    CLIMBING, // BLINK BLUE
    PARTY_MODE, // RAINBOW ANIMATION
    HOME // RED
  }

  public Lights() {
    
    config.LED.StripType = StripTypeValue.RGB;
    config.LED.BrightnessScalar = 1.0;

    candle.getConfigurator().apply(config);

    lightOptionsMap = new TreeMap<LightCode, RGBWColor>();
    lightOptionsMap.put(LightCode.OFF, new RGBWColor(0, 0, 0));
    lightOptionsMap.put(LightCode.DISABLED, new RGBWColor(255, 128, 0));
    lightOptionsMap.put(LightCode.INTAKING, new RGBWColor(255, 0, 0));
    lightOptionsMap.put(LightCode.AUTO_LOCKED, new RGBWColor(0, 255, 0));
    lightOptionsMap.put(LightCode.CLIMBING, new RGBWColor(0, 0, 255));
    lightOptionsMap.put(LightCode.HOME, new RGBWColor(255, 0, 0));

  }

  public void setLEDColor(LightCode light) {
    currentLightStatus = light;
    setLEDs();
  }

  private void setLEDs() {
    if (currentLightStatus == LightCode.PARTY_MODE 
        || currentLightStatus == LightCode.READY_TO_CLIMB) {
      setRainbow();
    } else if (currentLightStatus == LightCode.CLIMBING) {
      setBlink(currentLightStatus);
    } else {
      setSolid(currentLightStatus);
    }
  }

  private void setSolid(LightCode code) {
    SolidColor color = 
        new SolidColor(0, LightsCal.NUM_CANDLE_LEDS);

    color.withColor(lightOptionsMap.get(code));

    candle.setControl(color);
  }

  private void setBlink(LightCode code) {
    StrobeAnimation twinkle =
        new StrobeAnimation(0, LightsCal.NUM_CANDLE_LEDS);

    twinkle.withColor(lightOptionsMap.get(code));
    twinkle.withFrameRate(LightsCal.LIGHT_FRAMERATE);

    candle.setControl(twinkle);
  }

  private void setRainbow() {
    RainbowAnimation rainbow =
        new RainbowAnimation(0, LightsCal.NUM_CANDLE_LEDS);

    rainbow.withBrightness(LightsCal.LIGHT_BRIGHTNESS);
    rainbow.withFrameRate(LightsCal.LIGHT_FRAMERATE);

    candle.setControl(rainbow);
  }
}
