package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;



public class StartPrepShootSequence extends SequentialCommandGroup {
    public StartPrepShootSequence(Shooter shooter, Lights lights) {
        addRequirements(shooter);

        addCommands(
            /* TODO: Add variable shoot speed */
            /* TODO: Add hood pitch */
            new InstantCommand(() -> lights.setLEDColor(LightCode.SHOOT_PREPPING)),
            new InstantCommand(() -> shooter.runRollers(1.0)),
            new WaitUntilCommand(shooter::atDesiredSpeed),
            new InstantCommand(() -> lights.setLEDColor(LightCode.SHOOT_PREPPED))

        );
    }
}
