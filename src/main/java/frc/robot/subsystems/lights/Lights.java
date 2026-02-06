package frc.robot.subsystems.lights;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.util.Color;
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
    HOME, // PURPLE
    HOMING, // BLINKING PURLE
    ALIGNED, // RAINBOW
    ALIGNING, // GREEN
    SHOOT_PREPPED, // GREEN
    SHOOT_PREPPING, // BLINKING GREEN
    CLIMB_PREPPING, // BLINKING BLUE
    CLIMB_PREPPED, // BLUE
    CLIMBING, // RAINBOW
    INTAKING, // ORANGE
    AUTO_INTAKING, // BLINKING ORANGE
    FEEDING_PREPPING, // BLINKING GREEN
    FEEDING_PREPPED // GREEN
  }

  public Lights() {

    config.LED.StripType = StripTypeValue.RGB;
    config.LED.BrightnessScalar = 1.0;

    candle.getConfigurator().apply(config);

    lightOptionsMap = new TreeMap<LightCode, RGBWColor>();
    lightOptionsMap.put(LightCode.OFF, new RGBWColor(Color.kBlack));
    lightOptionsMap.put(LightCode.HOME, new RGBWColor(Color.kPurple));
    lightOptionsMap.put(LightCode.HOMING, new RGBWColor(Color.kPurple));
    lightOptionsMap.put(LightCode.ALIGNING, new RGBWColor(Color.kGreen));
    lightOptionsMap.put(LightCode.SHOOT_PREPPED, new RGBWColor(Color.kGreen));
    lightOptionsMap.put(LightCode.SHOOT_PREPPING, new RGBWColor(Color.kGreen));
    lightOptionsMap.put(LightCode.CLIMB_PREPPING, new RGBWColor(Color.kBlue));
    lightOptionsMap.put(LightCode.CLIMB_PREPPED, new RGBWColor(Color.kBlue));
    lightOptionsMap.put(LightCode.INTAKING, new RGBWColor(Color.kOrange));
    lightOptionsMap.put(LightCode.AUTO_INTAKING, new RGBWColor(Color.kOrange));
    lightOptionsMap.put(LightCode.FEEDING_PREPPED, new RGBWColor(Color.kGreen));
    lightOptionsMap.put(LightCode.FEEDING_PREPPING, new RGBWColor(Color.kGreen));
  }

  public void setLEDColor(LightCode light) {
    currentLightStatus = light;
    setLEDs();
  }

  private void setLEDs() {
    if (currentLightStatus == LightCode.ALIGNED ||
        currentLightStatus == LightCode.CLIMBING) {
      setRainbow();
    } else if (currentLightStatus == LightCode.HOMING ||
               currentLightStatus == LightCode.SHOOT_PREPPING ||
               currentLightStatus == LightCode.CLIMB_PREPPING ||
               currentLightStatus == LightCode.AUTO_INTAKING ||
               currentLightStatus == LightCode.FEEDING_PREPPING ) {
      setBlink(currentLightStatus);
    } else {
      setSolid(currentLightStatus);
    }
  }

  private void setSolid(LightCode code) {
    SolidColor color = new SolidColor(0, LightsCal.NUM_CANDLE_LEDS);

    color.withColor(lightOptionsMap.get(code));

    candle.setControl(color);
  }

  private void setBlink(LightCode code) {
    StrobeAnimation twinkle = new StrobeAnimation(0, LightsCal.NUM_CANDLE_LEDS);

    twinkle.withColor(lightOptionsMap.get(code));
    twinkle.withFrameRate(LightsCal.LIGHT_FRAMERATE);

    candle.setControl(twinkle);
  }

  private void setRainbow() {
    RainbowAnimation rainbow = new RainbowAnimation(0, LightsCal.NUM_CANDLE_LEDS);

    rainbow.withBrightness(LightsCal.LIGHT_BRIGHTNESS);
    rainbow.withFrameRate(LightsCal.LIGHT_FRAMERATE);

    candle.setControl(rainbow);
  }
}
