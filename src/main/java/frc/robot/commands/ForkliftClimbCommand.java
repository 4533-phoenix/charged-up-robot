package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Extension.ExtensionState;
import frc.robot.Robot;

public class ForkliftClimbCommand extends SequentialCommandGroup {
    public ForkliftClimbCommand(Extension extension, Climber climber) {
        addRequirements(extension, climber);
        addCommands(
            new InstantCommand(() -> extension.updateExtensionState(ExtensionState.GROUND_HIGH_INTAKE), extension), 
            new InstantCommand(() -> climber.raiseClimber(), climber),
            new WaitUntilCommand(() -> Robot.robotContainer.getSwerve().getPitch() > 80.0),
            new InstantCommand(() -> climber.stopClimber(), climber)
        );
    }
}