package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;

public class StartShoot extends SequentialCommandGroup {
    public StartShoot(Shooter shooter) {
        addRequirements(shooter);

        addCommands(
            /* TODO: Add variable shoot speed */
            /* TODO: Add hood pitch */
            new InstantCommand(() -> shooter.runRollers(1.0)) 
        );
    }
}
