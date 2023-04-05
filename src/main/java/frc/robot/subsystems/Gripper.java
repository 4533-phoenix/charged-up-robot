package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.*;


public final class Gripper implements Subsystem {
    private final Solenoid gripperCylinder = new Solenoid(PneumaticsModuleType.CTREPCM, GripperConstants.GRIPPER_PCM_PORT);

    private final DigitalInput gripperLimitSwitch = new DigitalInput(GripperConstants.LIMIT_SWITCH_PORT);

    public Gripper() {}

    public void enableGripper() {
        gripperCylinder.set(false);
    }

    public void disableGripper() {
        gripperCylinder.set(true);
    }

    public boolean objectInGripper() {
        return !gripperLimitSwitch.get();
    }

    @Override
    public void periodic() {}
}
