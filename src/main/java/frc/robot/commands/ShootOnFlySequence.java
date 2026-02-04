package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utils.ShootOnMoveUtil;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;

public class ShootOnFlySequence extends SequentialCommandGroup{
    

    public ShootOnFlySequence(Turret turret, Shooter shooter, Supplier<Pose2d> robotPoseSupplier, Supplier<Double> headingSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier, boolean isBlue, Lights lights){
        addRequirements(shooter, turret);

        addCommands(
            new RepeatCommand(
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                    Pair<Double, Double> results = ShootOnMoveUtil.calcTurret(isBlue, robotPoseSupplier.get(), chassisSpeedsSupplier.get(), headingSupplier.get());
                    shooter.setDesiredHoodPosition(results.getFirst());
                    turret.setDesiredTurretPosition(results.getSecond());
                }),
                new ConditionalCommand(
                    new InstantCommand(() -> lights.setLEDColor(LightCode.ALIGNED)),
                    new InstantCommand(() -> lights.setLEDColor(LightCode.ALIGNING)),
                    (() -> shooter.atDesiredHoodPosition() & turret.atDesiredTurretPosition()))
                )
            ).finallyDo(() -> lights.setLEDColor(LightCode.HOME))
        );
    }
}
