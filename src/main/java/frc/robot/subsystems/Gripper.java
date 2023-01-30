package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.AnalogInput;

import frc.libs.java.actionLib.*;
import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.controls.PSController.*;

public class Gripper extends Subsystem {
    public static Gripper mInstance;

    private final Solenoid gripperCylinder = new Solenoid(PneumaticsModuleType.CTREPCM, GripperConstants.GRIPPER_PCM_PORT);

    private final AnalogInput distanceSensor = new AnalogInput(GripperConstants.DISTANCE_SENSOR_PORT);

    private boolean isDroppingObject = false;

    private Gripper() {}

    public static Gripper getInstance() {
        if (mInstance == null) {
            mInstance = new Gripper();
        }
        return mInstance;
    }

    public void enableGripper() {
        this.isDroppingObject = false;

        gripperCylinder.set(true);
    }

    public void disableGripper() {
        this.isDroppingObject = true;

        gripperCylinder.set(false);
    }

    public boolean isDroppingObject() {
        return this.isDroppingObject;
    }

    public boolean objectInGripper() {
        return distanceSensor.getVoltage() > GripperConstants.DISTANCE_VOLTAGE_THRESHOLD;
    }

    private static final class GripperActions {
        public static final Action defaultGripperAction() {
            Runnable startMethod = () -> {};

            Runnable runMethod = () -> {
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

                    if (Robot.driverController.getButton(Button.X)) {
                        Gripper.getInstance().enableGripper();
                    } 
                    else if (Robot.driverController.getButton(Button.Y)) {
                        Gripper.getInstance().disableGripper();
                    }

                    System.out.println("voltage: " + Gripper.getInstance().distanceSensor.getVoltage());
            };

            Runnable endMethod = () -> {
                Gripper.getInstance().disableGripper();
            };

            return new Action(startMethod, runMethod, endMethod, false).withSubsystem(Gripper.getInstance());
        }
    }

    @Override
    public void log() {}

    @Override
    public void periodic() {}

    @Override
    public void queryInitialActions() {
        Robot.teleopRunner.add(
            GripperActions.defaultGripperAction()
        );
    }
}
