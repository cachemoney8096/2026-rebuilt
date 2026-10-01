package frc.robot.commands;

import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;

public class StartShootSequence extends SequentialCommandGroup {

    public StartShootSequence(Indexer indexer, Lights lights) {
        addRequirements(indexer);
        addCommands(
            new InstantCommand(() -> indexer.runIndexer()),
            new InstantCommand(() -> indexer.runKicker())
        );
    }
    
}
