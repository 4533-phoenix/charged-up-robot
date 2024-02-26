package frc.robot.commands;

import frc.robot.controls.PSController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Extension.ExtensionState;

public class DefaultExtensionCommand extends Command {
    private final Extension mExtension;

    public DefaultExtensionCommand(Extension extension) {
        mExtension = extension;

        addRequirements(extension);
    }

    @Override
    public void execute() {
        if (Robot.operatorController.getButton(Button.Y)) {
            mExtension.updateExtensionState(ExtensionState.GROUND_LOW_INTAKE);
        } else if (Robot.operatorController.getButton(Button.X) || Robot.operatorController.getButton(Button.BACK)) {
            mExtension.updateExtensionState(ExtensionState.OFF_GROUND);
        } else if (Robot.operatorController.getButton(Button.B)) {
            mExtension.updateExtensionState(ExtensionState.MIDDLE_ROW);
        } else if (Robot.operatorController.getButton(Button.A)) {
            mExtension.updateExtensionState(ExtensionState.HIGH_ROW);
        } else if (Robot.operatorController.getButton(Button.RB)) {
            mExtension.updateExtensionState(ExtensionState.HIGHER);
        } else if (Robot.operatorController.getButton(Button.LB)) {
            mExtension.updateExtensionState(ExtensionState.LOWER);
        } else if (Robot.operatorController.getButton(Button.START)) {
            mExtension.updateExtensionState(ExtensionState.KNOCKDOWN);
        } else {
            mExtension.updateExtensionState();
        }

        mExtension.updateElbowController();
    }
}
