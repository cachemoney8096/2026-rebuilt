package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.indexer.Indexer;


public class StartKickerSequence extends SequentialCommandGroup {

    public StartKickerSequence(Indexer indexer) {
        addRequirements(indexer);
        addCommands(
            new InstantCommand(() -> indexer.runIndexer()),
            new InstantCommand(() -> indexer.runKicker())
        );

    }
    
}
