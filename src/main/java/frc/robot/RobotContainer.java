package frc.robot;

import frc.robot.subsystems.*;

public final class RobotContainer {
    public static final void queryInitialActions() {
        Auto.getInstance().queryInitialActions();

        Swerve.getInstance().queryInitialActions();

        PoseEstimator.getInstance().queryInitialActions();

        Pneumatics.getInstance().queryInitialActions();

        Gripper.getInstance().queryInitialActions();

        Extension.getInstance().queryInitialActions();
    }
}
