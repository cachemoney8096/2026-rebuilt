package frc.robot.utils;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;

public class ShotTimeUtil {
    public static InterpolatingDoubleTreeMap timeFromDist = new InterpolatingDoubleTreeMap();

    public static void init(){

    }

    public static double getTimeFromDistance(double distance){
        return timeFromDist.get(distance);
    }
}