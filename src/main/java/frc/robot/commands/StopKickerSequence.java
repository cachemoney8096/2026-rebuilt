package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.indexer.Indexer;


public class StopKickerSequence extends SequentialCommandGroup {

    public StopKickerSequence(Indexer indexer) {
        addRequirements(indexer);
        addCommands(
            new InstantCommand(() -> indexer.stopIndexer()),
            new InstantCommand(() -> indexer.stopKicker())
        );

    }
    
}