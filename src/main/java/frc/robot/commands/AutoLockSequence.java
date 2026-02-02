package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utils.ShootOnMoveUtil;

public class AutoLockSequence extends SequentialCommandGroup{
    

    public AutoLockSequence(Turret turret, Lights lights, Shooter shooter, Supplier<Pose2d> robotPoseSupplier, Supplier<Double> headingSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier, boolean isBlue){
        addRequirements(shooter, turret);

        addCommands(
            new InstantCommand(() -> lights.setLEDColor(LightCode.AUTO_LOCKED)),
            new RepeatCommand(
                new InstantCommand(() -> {
                    
                    Pair<Double, Double> results = ShootOnMoveUtil.calcTurret(isBlue, robotPoseSupplier.get(), chassisSpeedsSupplier.get(), headingSupplier.get());

                    shooter.setDesiredHoodPosition(results.getFirst());
                    turret.setDesiredTurretPosition(results.getSecond());
                })
            ).finallyDo(
                new InstantCommand(() -> lights.setLEDColor(LightCode.OFF))
            )
        );
    }
}
