package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
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

    public void printColorRatiosRed() {
        System.out.println("Green Ratio: " + (double) colorSensor.getGreen()/colorSensor.getRed());
        System.out.println("Red Ratio (Should be 1): " + (double) colorSensor.getRed()/colorSensor.getRed());
        System.out.println("Blue Ratio: " + (double) colorSensor.getBlue()/colorSensor.getRed());
    }

    public void printColorRatiosBlue() {
        System.out.println("Green Ratio: " + (double) colorSensor.getGreen()/colorSensor.getBlue());
        System.out.println("Red Ratio: " + (double) colorSensor.getRed()/colorSensor.getBlue());
        System.out.println("Blue Ratio (Should be 1): " + (double) colorSensor.getBlue()/colorSensor.getBlue());
    }

    public void printColorRatiosGreen() {
        System.out.println("Green Ratio (Should be 1): " + (double) colorSensor.getGreen()/colorSensor.getGreen());
        System.out.println("Red Ratio: " + (double) colorSensor.getRed()/colorSensor.getGreen());
        System.out.println("Blue Ratio: " + (double) colorSensor.getBlue()/colorSensor.getGreen());
    }

    public void printObject() {
        if (objectInGripper() == true) {
            if (colorSensor.getGreen()/colorSensor.getBlue() > 2.1) {
                System.out.println("Cone");
            }
            else {
                System.out.println("Cube");
            }
        }
        else {
            System.out.println("Nothing");
        }
    }

    private static final class GripperActions {
        public static final Action defaultGripperAction() {
            Runnable startMethod = () -> {};

            Runnable runMethod = () -> {
                    //  double dropTime = 0;
                    //  double timestamp = Timer.getFPGATimestamp();

                    // if (Robot.driverController.getButton(Button.A)) {
                    //     if (!Gripper.getInstance().isDroppingObject) {
                    //         Gripper.getInstance().dropObject(timestamp);
                    //         dropTime = timestamp;
                    //     }
                    // } else if (Gripper.getInstance().isDroppingObject && Timer.getFPGATimestamp() - dropTime > 0.25) {
                    //     Gripper.getInstance().isDroppingObject = false;
                    // } else if (!Gripper.getInstance().isDroppingObject && Gripper.getInstance().objectInGripper()) {
                    //     Gripper.getInstance().enableGripper();
                    // } else {
                    //     Gripper.getInstance().disableGripper();
                    // }

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


                    if (Robot.driverController.getButton(Button.X)) {
                        Gripper.getInstance().enableGripper();
                    } else if (Robot.driverController.getButton(Button.Y)) {
                        Gripper.getInstance().disableGripper();
                    }

                    //System.out.println("voltage: " + Gripper.getInstance().distanceSensor.getVoltage());
                    //Gripper.getInstance().printObject();
            };

            Runnable endMethod = () -> {
                Gripper.getInstance().disableGripper();
            };

            return new Action(startMethod, runMethod, endMethod, ActionConstants.WILL_NOT_CANCEL).withSubsystem(Gripper.getInstance());
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
