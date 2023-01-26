package frc.robot.subsystems;

import frc.robot.Robot;
import frc.libs.java.actionLib.*;

public class Extension extends Subsystem {
    public static Extension mInstance;

    private Extension() {}

    public static Extension getInstance() {
        if (mInstance == null) {
            mInstance = new Extension();
        }
        return mInstance;
    }

    private static final class ExtensionActions {
        public static final Action defaultExtensionAction() {
            Runnable startMethod = () -> {};

            Runnable runMethod = () -> {};

            Runnable endMethod = () -> {};

            return new Action(startMethod, runMethod, endMethod, false).withSubsystem(Extension.getInstance());
        }
    }

    @Override
    public void log() {}

    @Override
    public void periodic() {}

    @Override
    public void queryInitialActions() {
        Robot.teleopRunner.add(
            ExtensionActions.defaultExtensionAction()
        );
    }
}