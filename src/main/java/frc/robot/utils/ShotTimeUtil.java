package frc.robot.utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShotTimeUtil {
    public static InterpolatingDoubleTreeMap timeFromDist = new InterpolatingDoubleTreeMap();

    public static void init(){

    }

    public static double getTimeFromDistance(double distance){
        return timeFromDist.get(distance);
    }
}
