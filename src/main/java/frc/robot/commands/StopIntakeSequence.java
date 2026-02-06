package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;

public class StopIntakeSequence extends SequentialCommandGroup{

    public StopIntakeSequence(Intake intake, Lights lights) {
    addRequirements(intake);

    addCommands(
        new InstantCommand(() -> intake.stopRollers()),
        new InstantCommand(() -> intake.setDesiredSlapdownPosition(IntakePosition.HOME)),
        new InstantCommand(()->lights.setLEDColor(LightCode.HOME))
    );
  }
    
}

