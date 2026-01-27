package frc.robot.commands;

import frc.robot.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;


public class StopShoot extends Command {

    private Shooter shooter;
    public StopShoot(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter); 
    }
    
    @Override
    public void execute() {
        shooter.stopRollers();
    }
}
