package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;

public class StopShoot extends SequentialCommandGroup {
    public StopShoot(Shooter shooter) {
        addRequirements(shooter);

        addCommands(
            new InstantCommand(() -> shooter.stopRollers())
        );
    }
}
