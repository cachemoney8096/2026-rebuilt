package frc.robot.utils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShootOnMoveUtil {
    /*
     * The goal of this util is to calculate the heading of the turret based on
     * distance to goal, robot velocity vector, and robot heading.
     * We can get away with making this simple by abusing the fact that the goal
     * is super wide, allowing us to treat airtime when we are shooting on the fly
     * as if it is the same as airtime when we are shooting stationary. Then, using
     * the velocity vector of the robot combined with that time, we can calculate
     * the effect that the robot's velocity has on the trajectory of the ball, which
     * we will apply as an offset to the target's effective position, which will
     * in turn adjust our turret heading accordingly. This heading will also account
     * for the robot's heading.
     */

    /**
     * @param isBlue    Is color blue
     * @param robotPose The current robot pose based on odometry
     * @param speeds    Robot chassis speeds
     * @param heading   Current robot heading in degrees
     * @return Pair of (turret pitch, turret heading)
     */

    public static Pair<Double, Double> calcTurret(boolean isBlue, Pose2d robotPose, ChassisSpeeds speeds,
            double heading) {

        // Initialize stuff
        Translation2d blueGoal = new Translation2d(4.6, 4);
        Translation2d redGoal = new Translation2d(11.9, 4);
        Translation2d robot = robotPose.getTranslation();
        double shootSpeedMPS = 7.5; // TODO this
        Translation2d goal = isBlue?blueGoal:redGoal;
        double turretRangeDeg = 180;

        // Emperical calibration constants (linear multiplier)
        double cdeg = 1.0; 
        double cpitch = 1.0;

        // Get some useful values
        Translation2d difference = goal.minus(robot);
        Pair<Double, Double> shooterData = ShooterPitchCalcUtil.calculate(shootSpeedMPS, new Pair<Double, Double>(Math.abs(difference.getX()), Math.abs(difference.getY())));
        double timeS = shooterData.getFirst();

        // Calculate the target offset
        double xVelocityFactor = -speeds.vxMetersPerSecond; 
        double yVelocityFactor = -speeds.vyMetersPerSecond;
        double targetOffsetX = xVelocityFactor*timeS;
        double targetOffsetY = yVelocityFactor*timeS;
        Translation2d targetOffsetTranslation = new Translation2d(targetOffsetX, targetOffsetY);
        goal = goal.plus(targetOffsetTranslation);

        // Calculate the absolute field heading to the target
        difference = goal.minus(robot);
        double angle = Math.toDegrees(Math.atan2(difference.getY(), difference.getX()));

        // Calculate the turret angle and pitch
        double headingDifference = 180 - (angle - heading + turretRangeDeg/2);
        shooterData = ShooterPitchCalcUtil.calculate(shootSpeedMPS, new Pair<Double, Double>(Math.abs(difference.getX()), Math.abs(difference.getY())));
        return new Pair<Double, Double>(Math.toDegrees(shooterData.getSecond())*cpitch, headingDifference*cdeg);
    }

    // Tester (ballparked numbers seem fine, can always use constants to tune)
    public static void main(String args[]){
        boolean isBlue = false;
        Pose2d robotPose = new Pose2d(14.0, 2.0, new Rotation2d());
        ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        double heading = 180.0;

        Pair<Double, Double> calcResult = calcTurret(isBlue, robotPose, speeds, heading);
        System.out.println("Pitch: " + calcResult.getFirst());
        System.out.println("Turret heading: " + calcResult.getSecond());
    }
}
