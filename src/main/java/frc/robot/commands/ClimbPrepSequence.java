package frc.robot.commands;

import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitUntilCommand;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbHeight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;


public class ClimbPrepSequence extends SequentialCommandGroup {

    public ClimbPrepSequence(Climb climb, Lights lights) {
        addRequirements(climb);
        addCommands(
            new InstantCommand(() -> lights.setLEDColor(LightCode.CLIMB_PREPPING)),
            new InstantCommand(() -> climb.setDesiredPosition(ClimbHeight.PREP)),
            new WaitUntilCommand(climb::atDesiredPosition),
            new InstantCommand(() -> lights.setLEDColor(LightCode.CLIMB_PREPPED))
        );
    }
    
}