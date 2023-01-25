package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.controls.DriveController;
import frc.robot.controls.PSController.Button;
import frc.robot.loops.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Extension extends Subsystem {
    public static Extension mInstance;

    private final DoubleSolenoid extensionCylinder = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
        ExtensionConstants.EXTENSION_PCM_PORT_FORWARD, ExtensionConstants.EXTENSION_PCM_PORT_REVERSE);

    public static Extension getInstance() {
        if (mInstance == null) {
            mInstance = new Extension();
        }
        return mInstance;
    }

    public void liftExtension() {
        extensionCylinder.set(Value.kForward);
    }

    public void dropExtension() {
        extensionCylinder.set(Value.kReverse);
    }

    public void extensionOff() {
        extensionCylinder.set(Value.kOff);
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
                public void onLoop(double timestamp) {
                    if (mController.getButton(Button.LB)) {
                        mExtension.liftExtension();
                    } else if (mController.getButton(Button.RB)) {
                        mExtension.dropExtension();
                    }
                }

                @Override
                public void onStop(double timestamp) {
                    mExtension.extensionOff();
                }
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