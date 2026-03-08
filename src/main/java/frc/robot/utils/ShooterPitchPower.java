package frc.robot.utils;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterPitchPower {
    private static InterpolatingDoubleTreeMap pitch = new InterpolatingDoubleTreeMap(); // <distance, pitch>
    private static InterpolatingDoubleTreeMap power = new InterpolatingDoubleTreeMap(); // <distance, power>

    public static void init(){
        pitch.put(0.0, 0.0);
        power.put(0.0, 0.0);
    }
    
    public static double getPitch(double distance){
        return pitch.get(distance);
    }

    public static double getPower(double distance){
        return power.get(distance);
    }
}
