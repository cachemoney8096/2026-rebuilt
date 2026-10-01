package frc.robot.commands;

import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;

public class StopPrepShootSequence extends SequentialCommandGroup {
    public StopPrepShootSequence(Shooter shooter, Lights lights) {
        addRequirements(shooter);

        addCommands(
            new InstantCommand(() -> shooter.stopRollers()),
            new InstantCommand(() -> lights.setLEDColor(LightCode.HOME))
            
        );
    }
}
