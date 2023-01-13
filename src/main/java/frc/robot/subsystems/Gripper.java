package frc.robot.subsystems;

import frc.robot.loops.*;

public class Gripper extends Subsystem {
    public static Gripper mInstance;

    public static Gripper getInstance() {
        if (mInstance == null) {
            mInstance = new Gripper();
        }
        return mInstance;
    }

    public Gripper() {}

    private static final class GripperLoops {}

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        GripperLoops gripperLoops = new GripperLoops();
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void writeToDashboard() {}
}
