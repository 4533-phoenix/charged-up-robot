package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.libs.java.actions.*;
import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.controls.PSController.*;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C.Port;

public final class Gripper extends Subsystem {
    private static Gripper mInstance;

    private final Solenoid gripperCylinder = new Solenoid(PneumaticsModuleType.CTREPCM, GripperConstants.GRIPPER_PCM_PORT);

    private final AnalogInput distanceSensor = new AnalogInput(GripperConstants.DISTANCE_SENSOR_PORT);

    private final ColorSensorV3 colorSensor = new ColorSensorV3(Port.kOnboard);

    private boolean isDroppingObject = true;

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

    public void dropObject(double timestamp) {
        if (objectInGripper()) {
            Gripper.getInstance().disableGripper();
            this.isDroppingObject = true;

            while (Timer.getFPGATimestamp() - 0.25 < timestamp) {
                System.out.println("dropping");
            }

            this.isDroppingObject = false;
        }
    }

    public boolean isDroppingObject() {
        return this.isDroppingObject;
    }

    public boolean objectInGripper() {
        return distanceSensor.getVoltage() > GripperConstants.DISTANCE_VOLTAGE_THRESHOLD;
    }

    public void printColor() {
        System.out.println("Green: " + colorSensor.getGreen());
        System.out.println("Red: " + colorSensor.getRed());
        System.out.println("Blue: " + colorSensor.getBlue());
    }

    public String getObject() {
        if (objectInGripper() == true) {
            if (colorSensor.getGreen()/colorSensor.getBlue() > 2.1) {
                return "Cone";
            }
            else {
                return "Cube";
            }
        }
        else {
            return "Nothing";
        }
    }

    private static final class GripperActions {
        public static final Action defaultGripperAction() {
            Runnable startMethod = () -> {};

            Runnable runMethod = () -> {
                    // if (Gripper.getInstance().objectInGripper() == true) {
                    //     if (Gripper.getInstance().colorSensor.getGreen()/Gripper.getInstance().colorSensor.getBlue() > 2.1) {
                    //         System.out.println("CONE GRABBED");
                    //         Gripper.getInstance().enableGripper();
                    //     }
                    //     else {
                    //         System.out.println("CUBE GRABBED");
                    //         Gripper.getInstance().enableGripper();
                    //     }
                    // }
                    // else {
                    //     Gripper.getInstance().disableGripper();
                    // }

                    if (Robot.driverController.getButton(Button.Y)) {
                        Gripper.getInstance().disableGripper();
                    } else if (Gripper.getInstance().objectInGripper()) {
                        Gripper.getInstance().enableGripper();
                    }

                    // if (Robot.driverController.getButton(Button.X)) {
                    //     Gripper.getInstance().enableGripper();
                    // } else if (Robot.driverController.getButton(Button.Y)) {
                    //     Gripper.getInstance().disableGripper();
                    // }

                    //System.out.println("voltage: " + Gripper.getInstance().distanceSensor.getVoltage());
                    //Gripper.getInstance().printObject();
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
