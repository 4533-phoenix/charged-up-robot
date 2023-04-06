package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Extension.ExtensionState;

public class SetClimbPositionCommand extends SequentialCommandGroup {
    public SetClimbPositionCommand(Extension extension, Climber climber) {
        addRequirements(extension, climber);
        addCommands(
            new InstantCommand(() -> extension.updateExtensionState(ExtensionState.ABOVE_MATCH_START), extension), 
            new InstantCommand(() -> climber.releaseClimber(), climber)
        );
    }
}
