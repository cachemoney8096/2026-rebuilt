package frc.robot.commands;

import java.util.function.Supplier;

import org.wpilib.util.Pair;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitUntilCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.ShootOnMoveUtil;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;

public class HoodAlignmentSequence extends SequentialCommandGroup{
    
    public HoodAlignmentSequence(Shooter shooter, Supplier<Pose2d> robotPoseSupplier, Supplier<Double> headingSupplier, Supplier<ChassisVelocities> chassisSpeedsSupplier, boolean isBlue, Lights lights) {
        
        addRequirements(shooter);
        addCommands(
            new InstantCommand(() -> lights.setLEDColor(LightCode.ALIGNING)),
            new InstantCommand(() -> {
                Pair<Double, Double> results = ShootOnMoveUtil.calcTurret(isBlue, robotPoseSupplier.get(), chassisSpeedsSupplier.get(), headingSupplier.get());
                shooter.setDesiredHoodPosition(()->results.getFirst());
            }),
            new WaitUntilCommand(shooter::atDesiredHoodPosition),
            new InstantCommand(() -> lights.setLEDColor(LightCode.ALIGNED))
        );
    }
}