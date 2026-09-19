package frc.robot.utils;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterPitchPower {
    private static InterpolatingDoubleTreeMap pitch = new InterpolatingDoubleTreeMap(); // <distance, pitch>
    private static InterpolatingDoubleTreeMap power = new InterpolatingDoubleTreeMap(); // <distance, power>

    public static void init(){
        power.put(0.5, 3900.0);
        power.put(2.3, 4600.0);
        power.put(4.7, 5300.0);
        pitch.put(0.5, 45.0);
        pitch.put(2.3, 55.0);
        pitch.put(4.7, 65.0);
    }
    
    public static double getPitch(double distance){
        return pitch.get(distance);
    }

    public static double getPower(double distance){
        return power.get(distance); 
    }
}
