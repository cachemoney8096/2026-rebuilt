package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.shooter.Shooter;

public class StartShoot extends Command {
    private Shooter shooter;
    public StartShoot(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter); 
    }
    @Override
    public void execute() {
        //TODO find correct speed
        // Should probably be handled in the shooter subsytem tbh
        shooter.runRollers(1);
    }
}
