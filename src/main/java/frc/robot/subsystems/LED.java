package frc.robot.subsystems;

import frc.robot.Constants.*;

import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class LED implements Subsystem {
    public AddressableLED ledStrip;
    public AddressableLEDBuffer ledBuffer;

    public LED() {}

    public enum Colors {
        BLUE(153, 153, 2),
        YELLOW(66, 247, 245),
        PURPLE(140, 32, 137),
        BLANK(0, 0, 0);
    
        public final int r;
        public final int g;
        public final int b;
    
        Colors(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

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

    public void fillAllLEDs(Colors color) {
        IntStream.range(1, this.ledBuffer.getLength())
            .forEach(i -> this.ledBuffer.setRGB(i, color.r, color.g, color.b));
    }

    public void fillIntervalLEDs(int interval, Colors[] colors) {
        IntStream.iterate(0, i -> i + interval)
            .limit((ledBuffer.getLength() + interval - 1) / interval)
            .forEach(i -> {
                int colorIndex = i % colors.length;
                ledBuffer.setRGB(i, colors[colorIndex].r, colors[colorIndex].g, colors[colorIndex].b);
            });
    }

    public void setLEDState(LEDState state) {
        if (state.equals(LEDState.YELLOW_AND_BLUE)) {
            Colors[] colors = new Colors[] {
                Colors.YELLOW,
                Colors.BLUE
            };
            fillIntervalLEDs(2, colors);
        } else if (state.equals(LEDState.PURPLE)) {
            fillAllLEDs(Colors.PURPLE);
        } else if (state.equals(LEDState.YELLOW)) {
            fillAllLEDs(Colors.YELLOW);
        } else if (state.equals(LEDState.OFF)) {
            fillAllLEDs(Colors.BLANK);
        }
    
        this.ledStrip.setData(this.ledBuffer);
    }

    @Override
    public void periodic() {}
}