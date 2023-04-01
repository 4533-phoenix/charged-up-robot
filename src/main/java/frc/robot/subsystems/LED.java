package frc.robot.subsystems;

import frc.robot.Constants.*;

import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class LED implements Subsystem {
    public AddressableLED ledStrip;
    public AddressableLEDBuffer ledBuffer;

    public LEDState ledState;
    public int animationFrame = 0;
    public Colors[] currentColors;
    public AnimationType animationType;

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

    public enum AnimationType {
        SCROLL, NONE
    }

    public enum LEDState {
        YELLOW_AND_BLUE, YELLOW_AND_BLUE_ANIMATION, PURPLE, YELLOW, OFF
    }

    public void configureLEDs() {
        ledStrip = new AddressableLED(LEDConstants.LED_PWM_PORT);

        ledBuffer = new AddressableLEDBuffer(80);
        ledStrip.setLength(ledBuffer.getLength());

        this.setLEDState(LEDState.YELLOW_AND_BLUE_ANIMATION);

        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    public void fillLEDs(Colors[] colors) {
        int interval = colors.length;
        IntStream.range(0, ledBuffer.getLength())
            .forEach(i -> {
                int colorIndex = i % interval;
                ledBuffer.setRGB(i, colors[colorIndex].r, colors[colorIndex].g, colors[colorIndex].b);
            });
    }

    public void scrollAnimateLeds(Colors[] colors) {
        int interval = colors.length;
        IntStream.range(0, ledBuffer.getLength())
            .forEach(i -> {
                int colorIndex = (i + animationFrame) % interval;
                ledBuffer.setRGB(i, colors[colorIndex].r, colors[colorIndex].g, colors[colorIndex].b);
            });
        animationFrame = (animationFrame + 1) % interval;
    }

    public void setLEDState(LEDState state) {
        ledState = state;

        switch (ledState) {
            case YELLOW_AND_BLUE:
                animationType = AnimationType.NONE;
                currentColors = new Colors[] { Colors.YELLOW, Colors.BLUE };
                break;
            case PURPLE:
                animationType = AnimationType.NONE;
                currentColors = new Colors[] { Colors.PURPLE };
                break;
            case YELLOW:
                animationType = AnimationType.NONE;
                currentColors = new Colors[] { Colors.YELLOW };
                break;
            case OFF:
                animationType = AnimationType.NONE;
                currentColors = new Colors[] { Colors.BLANK };
                break;
            case YELLOW_AND_BLUE_ANIMATION:
                animationType = AnimationType.SCROLL;
                animationFrame = 0;
                currentColors = new Colors[] { Colors.YELLOW, Colors.BLUE, Colors.BLUE, Colors.BLUE, Colors.BLUE };
                break;
            default:
                break;
        }
    
        this.ledStrip.setData(this.ledBuffer);
    }

    @Override
    public void periodic() {
        if (currentColors.length > 0) {
            switch (animationType) {
                case NONE:
                    fillLEDs(currentColors);
                    break;
                case SCROLL:
                    scrollAnimateLeds(currentColors);
                    break;
                default:
                    break;
            }
        }

        ledStrip.setData(ledBuffer);
    }
}