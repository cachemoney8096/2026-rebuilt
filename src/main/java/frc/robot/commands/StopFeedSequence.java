package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.shooter.Shooter;

public class StopFeedSequence extends SequentialCommandGroup {
    
    public StopFeedSequence(Shooter shooter, Indexer indexer, Lights lights, Intake intake) {
        
        addCommands(
            new StopShoot(shooter),
            new StopIntakeSequence(intake, lights),
            new StopKickerSequence(indexer)
        );
    }
}
