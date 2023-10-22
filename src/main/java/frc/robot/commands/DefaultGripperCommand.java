package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Robot;
import frc.robot.controls.PSController.*;

public class DefaultGripperCommand extends CommandBase {
    private final Gripper mGripper;

    public DefaultGripperCommand(Gripper gripper) {
        mGripper = gripper;

        addRequirements(gripper);
    }

    @Override
    public void execute() {
        if (Robot.operatorController.getTrigger(Side.RIGHT)) {
            mGripper.enableGripper();
        } else if (Robot.driverController.getTrigger(Side.LEFT) || Robot.operatorController.getTrigger(Side.LEFT)) {
            mGripper.disableGripper();
        }
    }
}
