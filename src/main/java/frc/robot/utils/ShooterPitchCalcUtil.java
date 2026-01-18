package frc.robot.utils;

import edu.wpi.first.math.Pair;

public class ShooterPitchCalcUtil {
    private static final double GRAVITY = 9.80665;
    
    /**
     * 
     * @param velocity
     * @param distances (Delta X, Delta Y) from hub
     * @return Time, Theta
     */
    public static Pair<Double, Double> calculate(double velocity, Pair<Double, Double> distances) {
        double square_velocity = Math.pow(velocity, 2);

        double TERM1 = square_velocity - (GRAVITY * distances.getSecond());

        double TERM2_1 = Math.pow(square_velocity - (GRAVITY * distances.getSecond()), 2);
        double TERM2_2 = Math.pow(GRAVITY, 2) * (Math.pow(distances.getFirst(), 2) + Math.pow(distances.getSecond(), 2));

        double TERM2 = Math.sqrt(TERM2_1 - TERM2_2);

        double numerator_pos = 2 * (TERM1 + TERM2);
        // ignoring negative term for now..
        //double numerator_neg = 2 * (TERM1 - TERM2);

        double time = Math.sqrt((numerator_pos / (Math.pow(GRAVITY, 2))));

        double theta = Math.atan((distances.getSecond() + (0.5 * GRAVITY * Math.pow(time, 2))) / distances.getFirst());

        return new Pair<Double,Double>(time, theta);
    }
}