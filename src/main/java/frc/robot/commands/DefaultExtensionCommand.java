package frc.robot.commands;

import frc.robot.controls.PSController.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Extension.ExtensionState;

public class DefaultExtensionCommand extends CommandBase {
    private final Extension mExtension;

    public DefaultExtensionCommand(Extension extension) {
        mExtension = extension;

        addRequirements(extension);
    }

    @Override
    public void execute() {
        mExtension.updateExtensionState();
        mExtension.updateElbowController();
    }
}
