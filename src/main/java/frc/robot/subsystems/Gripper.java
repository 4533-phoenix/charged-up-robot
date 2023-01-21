package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.GripperConstants;
import frc.robot.controls.DriveController;
import frc.robot.controls.PSController.Button;
import frc.robot.loops.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AnalogInput;

public class Gripper extends Subsystem {
    public static Gripper mInstance;

    private final Solenoid gripperCylinder = new Solenoid(PneumaticsModuleType.CTREPCM, GripperConstants.GRIPPER_PCM_PORT);

    private final AnalogInput distanceSensor = new AnalogInput(GripperConstants.DISTANCE_SENSOR_PORT);

    public boolean isDroppingObject = false;

    public void enableGripper() {
        gripperCylinder.set(true);
    }

    public void disableGripper() {
        gripperCylinder.set(false);
    }

    public boolean objectInGripper() {
        return distanceSensor.getVoltage() > GripperConstants.DISTANCE_VOLTAGE_THRESHOLD;
    }

    public void dropObject(double timestamp) {
        if (!isDroppingObject && Timer.getFPGATimestamp() - timestamp < 0.25) {
            isDroppingObject = true;
            disableGripper();
        }
    }

    public static Gripper getInstance() {
        if (mInstance == null) {
            mInstance = new Gripper();
        }
        return mInstance;
    }

    public Gripper() {}

    private static final class GripperLoops {
        private Gripper mGripper = Gripper.getInstance();
        private DriveController mController = DriveController.getInstance();

        public Loop defaultGripperLoop() {
            return new Loop() {
                @Override
                public void onStart(double timestamp) {}

                @Override
                public void onLoop(double timestamp) {
                    // double dropTime = 0;

                    // if (mController.getButton(Button.A)) {
                    //     if (!mGripper.isDroppingObject) {
                    //         mGripper.dropObject(timestamp);
                    //         dropTime = timestamp;
                    //     }
                    // } else if (mGripper.isDroppingObject && Timer.getFPGATimestamp() - dropTime > 0.25) {
                    //     mGripper.isDroppingObject = false;
                    // } else if (!mGripper.isDroppingObject && mGripper.objectInGripper()) {
                    //     mGripper.enableGripper();
                    // } else {
                    //     mGripper.disableGripper();
                    // }

                    if (mController.getButton(Button.X)) {
                        mGripper.enableGripper();
                    } else if (mController.getButton(Button.Y)) {
                        mGripper.disableGripper();
                    }
                }

                @Override
                public void onStop(double timestamp) {
                    mGripper.disableGripper();
                }
            };
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        GripperLoops gripperLoops = new GripperLoops();

        mEnabledLooper.register(gripperLoops.defaultGripperLoop());
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
