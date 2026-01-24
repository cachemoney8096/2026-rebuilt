package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;


public class StartIntakeSequence extends SequentialCommandGroup{

    public StartIntakeSequence(
      Intake intake, Lights lights) {
    addRequirements(intake);

    addCommands(
        new InstantCommand(() -> lights.setLEDColor(LightCode.INTAKING)),
        new InstantCommand(() -> intake.setDesiredSlapdownPosition(IntakePosition.EXTENDED)),
        new WaitUntilCommand(intake::atDesiredSlapdownPosition),
        new InstantCommand(() -> intake.runRollers())

    );
  }
    
}
