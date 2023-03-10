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

    public enum LEDState {
        YELLOW_AND_BLUE, PURPLE, YELLOW, OFF
    }

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

        this.setLEDState(LEDState.YELLOW_AND_BLUE);

        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    public void setLEDState(LEDState state) {
        if (state.equals(LEDState.YELLOW_AND_BLUE)) {
            for (int i = 1; i < this.ledBuffer.getLength(); i += 2) {
                this.ledBuffer.setRGB(i - 1, 66, 247, 245);
                this.ledBuffer.setRGB(i, 153, 153, 2);
            }
        } else if (state.equals(LEDState.PURPLE)) {
            for (int i = 1; i < this.ledBuffer.getLength(); i++) {
                this.ledBuffer.setRGB(i, 140, 32, 137);
            }
        } else if (state.equals(LEDState.YELLOW)) {
            for (int i = 1; i < this.ledBuffer.getLength(); i++) {
                this.ledBuffer.setRGB(i, 153, 153, 2);
            }
        } else if (state.equals(LEDState.OFF)) {
            for (int i = 1; i < this.ledBuffer.getLength(); i++) {
                this.ledBuffer.setRGB(i, 0, 0, 0);
            }
        }

        this.ledStrip.setData(this.ledBuffer);
    }

    private static final class LEDActions {
        public static final Action defaultLEDAction() {
            Runnable startMethod = () -> {};

            Runnable runMethod = () -> {
                if (Robot.operatorController.getPOV() == 0) {
                    LED.getInstance().setLEDState(LEDState.YELLOW);
                } else if (Robot.operatorController.getPOV() == 90) {
                    LED.getInstance().setLEDState(LEDState.OFF);
                } else if (Robot.operatorController.getPOV() == 180) {
                    LED.getInstance().setLEDState(LEDState.PURPLE);
                } else if (Robot.operatorController.getPOV() == 270) {
                    LED.getInstance().setLEDState(LEDState.YELLOW_AND_BLUE);
                }
            };

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