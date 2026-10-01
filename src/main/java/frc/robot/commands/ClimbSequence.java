package frc.robot.commands;

import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbHeight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;



public class ClimbSequence extends SequentialCommandGroup {

    public ClimbSequence(Climb climb, Lights lights) {
        addRequirements(climb);
        addCommands(
            new InstantCommand(() -> lights.setLEDColor(LightCode.CLIMBING)),
            new InstantCommand(() -> climb.setDesiredPosition(ClimbHeight.FINISHED))
        );

    }
    
}