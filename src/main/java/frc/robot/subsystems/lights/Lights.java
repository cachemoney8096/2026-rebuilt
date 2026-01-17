package frc.robot.subsystems.lights;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import frc.robot.RobotMap;

import java.util.TreeMap;

public class Lights {
  /**
   * Tree map of LightCode enums and integers that represent the R, G, B values of each light code
   * enum
   */
  private TreeMap<LightCode, RGBWColor> lightOptionsMap;

  private CANdle candle = new CANdle(RobotMap.CANDLE_CAN_ID, "rio");
  private CANdleConfiguration config = new CANdleConfiguration();
  private LightCode currentLightStatus = LightCode.OFF;

  public enum LightCode {
    OFF, // BLACK
    DISABLED, // ORANGE
    READY_TO_INTAKE, // BLINK RED
    HAS_CORAL, // GREEN
    SCORE_PREP, // BLUE
    READY_TO_SCORE, // RAINBOW
    CLIMB_PREP_SHALLOW, // PURPLE
    CLIMB_PREP_DEEP, // BLINK PURPLE
    READY_TO_CLIMB, // RAINBOW
    CLIMBING, // BLINK BLUE
    PARTY_MODE, // RAINBOW ANIMATION
    HOME // RED
  }

  public Lights() {
    
    config.LED.StripType = StripTypeValue.RGB;
    config.LED.BrightnessScalar = 1.0;
    // Check this line
    candle.getConfigurator().apply(config);

    lightOptionsMap = new TreeMap<LightCode, RGBWColor>();
    lightOptionsMap.put(LightCode.OFF, new RGBWColor(0, 0, 0));

  }

  public void setLEDColor(LightCode light) {
    currentLightStatus = light;
    setLEDs();
  }

  private void setLEDs() {
    if (currentLightStatus == LightCode.PARTY_MODE
        || currentLightStatus == LightCode.READY_TO_CLIMB
        || currentLightStatus == LightCode.READY_TO_SCORE) {
      setRainbow();
    } else if (currentLightStatus == LightCode.CLIMBING
        || currentLightStatus == LightCode.CLIMB_PREP_DEEP
        || currentLightStatus == LightCode.READY_TO_INTAKE) {
      setBlink(currentLightStatus);
    } else {
      setSolid(currentLightStatus);
    }
  }

  private void setSolid(LightCode code) {
    SolidColor color = 
        new SolidColor(0, LightsCal.NUM_CANDLE_LEDS);

    color.withColor(lightOptionsMap.get(code));
  }

  private void setBlink(LightCode light) {
    StrobeAnimation twinkle =
        new StrobeAnimation(0, LightsCal.NUM_CANDLE_LEDS);
    candle.setControl(twinkle);
  }

  private void setRainbow() {
    RainbowAnimation rainbowAnim =
        new RainbowAnimation(0, LightsCal.NUM_CANDLE_LEDS);
    candle.setControl(rainbowAnim);
  }
}
