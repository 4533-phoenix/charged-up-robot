package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.libs.java.actions.*;
import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.controls.PSController.*;


public final class Gripper extends Subsystem {
    private static Gripper mInstance;

    private final Solenoid gripperCylinder = new Solenoid(PneumaticsModuleType.CTREPCM, GripperConstants.GRIPPER_PCM_PORT);

    private final DigitalInput gripperLimitSwitch = new DigitalInput(GripperConstants.LIMIT_SWITCH_PORT);

    private Gripper() {}

    public static Gripper getInstance() {
        if (mInstance == null) {
            mInstance = new Gripper();
        }

        return mInstance;
    }

    public void enableGripper() {
        gripperCylinder.set(false);
    }

    public void disableGripper() {
        gripperCylinder.set(true);
    }

    public boolean objectInGripper() {
        return !gripperLimitSwitch.get();
    }

    private static final class GripperActions {
        public static final Action defaultGripperAction() {
            Runnable startMethod = () -> {};

            Runnable runMethod = () -> {
                    if (Robot.driverController.getTrigger(Side.LEFT) || Robot.operatorController.getTrigger(Side.LEFT)) {
                        Gripper.getInstance().disableGripper();
                    } else if (Robot.driverController.getTrigger(Side.RIGHT) || Robot.operatorController.getTrigger(Side.RIGHT)) {
                        Gripper.getInstance().enableGripper();
                    } else if (Gripper.getInstance().objectInGripper()) {
                        Gripper.getInstance().enableGripper();
                    }
            };

            Runnable endMethod = () -> {
                Gripper.getInstance().disableGripper();
            };

            return new Action(startMethod, runMethod, endMethod, ActionConstants.WILL_NOT_CANCEL);
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
