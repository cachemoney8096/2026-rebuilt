package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.lights.Lights;


public class StopShootSequence extends SequentialCommandGroup {

    public StopShootSequence(Indexer indexer, Lights lights) {
        addRequirements(indexer);
        addCommands(
            new InstantCommand(() -> indexer.stopIndexer()),
            new InstantCommand(() -> indexer.stopKicker())
        );

    }
    
}