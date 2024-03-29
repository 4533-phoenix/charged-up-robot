package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class WaitForGamepieceCommand extends CommandBase {
    private final Gripper mGripper;

    public WaitForGamepieceCommand(Gripper gripper) {
        mGripper = gripper;

        addRequirements(gripper);
    }

    @Override
    public void execute() {
        if (mGripper.objectInGripper()) {
            mGripper.enableGripper();
        } 
    }

    @Override
    public boolean isFinished() {
        return mGripper.objectInGripper();
    }
}
