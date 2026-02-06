package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.ShootOnMoveUtil;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;

public class HoodAlignmentSequence extends SequentialCommandGroup{
    
    public HoodAlignmentSequence(Shooter shooter, Supplier<Pose2d> robotPoseSupplier, Supplier<Double> headingSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier, boolean isBlue, Lights lights) {
        
        addRequirements(shooter);
        addCommands(
            new InstantCommand(() -> lights.setLEDColor(LightCode.ALIGNING)),
            new InstantCommand(() -> {
                Pair<Double, Double> results = ShootOnMoveUtil.calcTurret(isBlue, robotPoseSupplier.get(), chassisSpeedsSupplier.get(), headingSupplier.get());
                shooter.setDesiredHoodPosition(results.getFirst());
            }),
            new WaitUntilCommand(shooter::atDesiredHoodPosition),
            new InstantCommand(() -> lights.setLEDColor(LightCode.ALIGNED))
        );
    }
}