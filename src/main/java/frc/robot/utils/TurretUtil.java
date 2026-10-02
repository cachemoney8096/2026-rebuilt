package frc.robot.utils;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;

public class TurretUtil {
    public static Translation2d blueTarget = new Translation2d(5.3, 3.5);
    public static Translation2d redTarget = new Translation2d(11.7, 4.0);

    public static double turretTargetHeading(BooleanSupplier isBlueBooleanSupplier, Supplier<Pose2d> robotPoseSupplier,
            DoubleSupplier headingSupplier){
                if(isBlueBooleanSupplier.getAsBoolean()){
                    return TurretUtil.turretTargetHeading(isBlueBooleanSupplier, robotPoseSupplier, headingSupplier, blueTarget);
                }
                else{
                    return TurretUtil.turretTargetHeading(isBlueBooleanSupplier, robotPoseSupplier, headingSupplier, redTarget);
                }
            }

    public static double turretTargetHeading(BooleanSupplier isBlueBooleanSupplier, Supplier<Pose2d> robotPoseSupplier,
            DoubleSupplier headingSupplier, Translation2d target) {
        if (isBlueBooleanSupplier.getAsBoolean()) {
            double angle = Math.toDegrees(
                    Math.atan2(robotPoseSupplier.get().getY() - target.getY(),
                            robotPoseSupplier.get().getX() - target.getX()));
            if (angle >= 0) {
                angle = 90 + (180 - angle);
            } else {
                angle = 90 - (180 + angle);
            }
            return angle + headingSupplier.getAsDouble();
        } else {
            double angle = Math.toDegrees(
                    Math.atan2(robotPoseSupplier.get().getY() - target.getY(),
                            robotPoseSupplier.get().getX() - target.getX()));
            if (angle >= 0) {
                angle = 90 + (180 - angle);
            } else {
                angle = 90 - (180 + angle);
            }
            return MathUtil.inputModulus(
                    MathUtil.inputModulus(angle + 180.0, 0.0, 360.0) + headingSupplier.getAsDouble() % 360 + 180.0, 0.0,
                    360.0);
        }
    }

    public static double turretTargetHeadingConsidersRobotVelocity(BooleanSupplier isBlueBooleanSupplier,
            Supplier<Pose2d> robotPoseSupplier,
            DoubleSupplier headingSupplier, Supplier<ChassisVelocities> speeds) {
        Translation2d target = isBlueBooleanSupplier.getAsBoolean()?blueTarget:redTarget;
        Translation2d robot = robotPoseSupplier.get().getTranslation();
        double distance = Math.abs(target.getDistance(robot));
        double time = 1.2;
        double xOffset = -speeds.get().vx*time;
        double yOffset = -speeds.get().vy*time;
        if(isBlueBooleanSupplier.getAsBoolean()){
            return TurretUtil.turretTargetHeading(isBlueBooleanSupplier, robotPoseSupplier, headingSupplier, blueTarget.plus(new Translation2d(xOffset, yOffset)));
        }
        else{
            return TurretUtil.turretTargetHeading(isBlueBooleanSupplier, robotPoseSupplier, headingSupplier, redTarget.plus(new Translation2d(xOffset, yOffset)));
        }
    }
}