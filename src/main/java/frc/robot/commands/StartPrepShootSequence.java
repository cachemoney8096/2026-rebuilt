package frc.robot.commands;

import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;
import org.wpilib.command2.WaitUntilCommand;



public class StartPrepShootSequence extends SequentialCommandGroup {
    public StartPrepShootSequence(Shooter shooter, Lights lights) {
        addRequirements(shooter);

        addCommands(
            new InstantCommand(() -> lights.setLEDColor(LightCode.SHOOT_PREPPING)),
            new InstantCommand(() -> shooter.runRollers()),
            new WaitUntilCommand(shooter::atDesiredSpeed),
            new InstantCommand(() -> lights.setLEDColor(LightCode.SHOOT_PREPPED))

        );
    }
}
