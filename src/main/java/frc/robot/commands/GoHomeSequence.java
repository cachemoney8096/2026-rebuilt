package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretCal;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbHeight;
import frc.robot.subsystems.indexer.Indexer;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;


public class GoHomeSequence extends SequentialCommandGroup{
    public GoHomeSequence(Turret turret, Intake intake, Climb climb, Shooter shooter, Indexer indexer, Lights lights) {
        addRequirements(turret, intake, climb, shooter, indexer);
        addCommands(
            new InstantCommand(() -> lights.setLEDColor(LightCode.HOMING)),
            new InstantCommand(() -> shooter.setDesiredHoodPosition(0)),
            new InstantCommand(() -> intake.stopRollers()),
            new InstantCommand(() -> intake.setDesiredSlapdownPosition(IntakePosition.HOME)),
            new InstantCommand(() -> climb.setDesiredPosition(ClimbHeight.HOME)),
            new InstantCommand(() -> turret.setDesiredTurretPosition(TurretCal.TURRET_HOME_DEGREES)),
            new InstantCommand(() -> indexer.stopKicker()),
            new InstantCommand(() -> indexer.stopIndexer()),
            new WaitUntilCommand(shooter::atDesiredHoodPosition),
            new WaitUntilCommand(intake::atDesiredSlapdownPosition),
            new WaitUntilCommand(climb::atDesiredPosition),
            new WaitUntilCommand(turret::atDesiredTurretPosition),
            new WaitUntilCommand(() -> !indexer.indexerIsOn()),
            new WaitUntilCommand(() -> !indexer.kickerIsOn()),
            new InstantCommand(() -> lights.setLEDColor(LightCode.HOME))
        );
    }
}
