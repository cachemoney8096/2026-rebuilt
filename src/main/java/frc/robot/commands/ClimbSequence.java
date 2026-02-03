package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbHeight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;


public class ClimbSequence extends SequentialCommandGroup {

    public ClimbSequence(Climb climb, Lights lights) {
        addRequirements(climb);
        addCommands(
            new InstantCommand(() -> lights.setLEDColor(LightCode.CLIMBING)),
            new InstantCommand(() -> climb.setDesiredPosition(ClimbHeight.L1_TELEOP))
        );

    }
    
}