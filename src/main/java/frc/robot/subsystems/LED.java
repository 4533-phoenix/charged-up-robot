package frc.robot.subsystems;

import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class LED implements Subsystem {
    public AddressableLED ledStrip;
    public AddressableLEDBuffer ledBuffer;

    public LED() {}

    public enum LEDState {
        YELLOW_AND_BLUE, PURPLE, YELLOW, OFF
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

    @Override
    public void periodic() {}
}