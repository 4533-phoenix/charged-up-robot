package frc.robot.subsystems;

import frc.libs.java.actions.*;
import frc.robot.Robot;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public final class LED extends Subsystem {
    private static LED mInstance;

    public AddressableLED ledStrip;
    public AddressableLEDBuffer ledBuffer;

    private LED() {}

    public static LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }

        return mInstance;
    }

    public void configureLEDs() {
        ledStrip = new AddressableLED(LEDConstants.LED_PWM_PORT);

        ledBuffer = new AddressableLEDBuffer(80);
        ledStrip.setLength(ledBuffer.getLength());

        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    private static final class LEDActions {
        public static final Action defaultLEDAction() {
            Runnable startMethod = () -> {};

            Runnable runMethod = () -> {};

            Runnable endMethod = () -> {};

            return new Action(startMethod, runMethod, endMethod, ActionConstants.WILL_NOT_CANCEL);
        }
    }

    @Override
    public void log() {}

    @Override
    public void periodic() {}

    @Override
    public void queryInitialActions() {
        Robot.autonomousRunner.add(
            LEDActions.defaultLEDAction()
        );

        Robot.teleopRunner.add(
            LEDActions.defaultLEDAction()
        );
    }
}