package frc.robot.utils;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterPitchPower {
    private static InterpolatingDoubleTreeMap pitch = new InterpolatingDoubleTreeMap(); // <distance, pitch>
    private static InterpolatingDoubleTreeMap power = new InterpolatingDoubleTreeMap(); // <distance, power>

    public static void init(){
        power.put(1.8, 3900.0);
        power.put(2.1, 4100.0);
        power.put(2.4, 4400.0);
        power.put(2.7, 4500.0);
        power.put(3.0, 4600.0);
        power.put(3.5, 5000.0);
        power.put(4.5, 5100.0);
        power.put(5.0, 5200.0);
        pitch.put(1.8, 45.0);
        pitch.put(2.1, 45.0);
        pitch.put(2.4, 45.0);
        pitch.put(2.7, 45.0);
        pitch.put(3.0, 45.0);
        pitch.put(3.5, 45.0);
        pitch.put(4.5, 55.0);
        pitch.put(5.0, 60.0);
    }
    
    public static double getPitch(double distance){
        return pitch.get(distance+1.8);
    }

    public static double getPower(double distance){
        return power.get(distance+1.8); 
    }
}
