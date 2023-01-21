package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.controls.DriveController;
import frc.robot.controls.PSController.Button;
import frc.robot.loops.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Extension extends Subsystem {
    public static Extension mInstance;

    public static Extension getInstance() {
        if (mInstance == null) {
            mInstance = new Extension();
        }
        return mInstance;
    }

    public Extension() {}

    private static final class ExtensionLoops {
        private Extension mExtension = Extension.getInstance();
        private DriveController mController = DriveController.getInstance();

        public Loop defaultExtensionLoop() {
            return new Loop() {
                @Override
                public void onStart(double timestamp) {}

                @Override
                public void onLoop(double timestamp) {}

                @Override
                public void onStop(double timestamp) {}
            };
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        ExtensionLoops extensionLoops = new ExtensionLoops();

        mEnabledLooper.register(extensionLoops.defaultExtensionLoop());
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