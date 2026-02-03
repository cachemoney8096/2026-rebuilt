package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretCal;

public class FeedSequence extends SequentialCommandGroup {
    double targetAngle = (TurretCal.TURRET_MAX_DEGREES - TurretCal.TURRET_MIN_DEGREES) / 2.0;
    
    public FeedSequence(Turret turret, Supplier<Double> headingSupplier, Boolean isBlue, Trigger cancelButton) {
        addRequirements(turret);
        addCommands(
            new RepeatCommand(new InstantCommand(() -> {
                double heading = headingSupplier.get();

                /* Converts from 0 to 360 to -180 to 180. Also flips if red. */
                double adjustedHeading = isBlue ? heading : heading + 180.0;
                adjustedHeading = adjustedHeading <= 180.0 ? adjustedHeading : adjustedHeading - 360.0;

                turret.setDesiredTurretPosition(targetAngle - adjustedHeading + 180);
            })).unless(cancelButton)
        );
    }
}



