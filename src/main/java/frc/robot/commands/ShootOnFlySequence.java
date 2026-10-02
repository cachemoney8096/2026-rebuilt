package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.wpilib.util.Pair;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.command2.ConditionalCommand;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utils.ShootOnMoveUtil;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;

public class ShootOnFlySequence extends SequentialCommandGroup{
    

    public ShootOnFlySequence(Turret turret, Shooter shooter, Supplier<Pose2d> robotPoseSupplier, Supplier<Double> headingSupplier, Supplier<ChassisVelocities> chassisSpeedsSupplier, BooleanSupplier isBlue, Lights lights){
        addRequirements(shooter, turret);
        double heading = headingSupplier.get();
        if(heading < 0){
            heading += 360;
        }
        addCommands(
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                    Pair<Double, Double> results = ShootOnMoveUtil.calcTurret(isBlue.getAsBoolean(), robotPoseSupplier.get(), chassisSpeedsSupplier.get(), headingSupplier.get());
                    //shooter.setDesiredHoodPosition(results.getFirst());
                    turret.setDesiredTurretPosition(results.getSecond());
                }),
                new ConditionalCommand(
                    new InstantCommand(() -> lights.setLEDColor(LightCode.ALIGNED)),
                    new InstantCommand(() -> lights.setLEDColor(LightCode.ALIGNING)),
                    (() -> shooter.atDesiredHoodPosition() && turret.atDesiredTurretPosition())
                )
            )
        );
    }
}
