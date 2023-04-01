package frc.robot.subsystems;

import frc.robot.Constants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class LED implements Subsystem {
    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;

    private LEDState ledState;
    private int animationFrame = 0;
    private int animationSpeed = 0;
    private int currentAnimationTime = 0;
    private Colors[] currentColors;
    private AnimationType animationType;

    public LED() {}

    private enum Colors {
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

    private enum AnimationType {
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

    private void resetAnimation() {
        animationFrame = 0;
        currentAnimationTime = 0;
    }

    private void setAnimation(int speed, AnimationType type) {
        resetAnimation();
        animationSpeed = speed;
        animationType = type;
    }

    private void fillLEDs(Colors[] colors) {
        int interval = ledBuffer.getLength() / colors.length;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            int colorIndex = (i / interval) % colors.length;
            ledBuffer.setRGB(i, colors[colorIndex].r, colors[colorIndex].g, colors[colorIndex].b);
        }
    }

    private void scrollAnimateLEDs(Colors[] colors) {
        int interval = colors.length;
        int startIndex = animationFrame % interval;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            int colorIndex = (i + startIndex) % interval;
            ledBuffer.setRGB(i, colors[colorIndex].r, colors[colorIndex].g, colors[colorIndex].b);
        }
        animationFrame = (animationFrame + 1) % interval;
    }

    public void setLEDState(LEDState state) {
        ledState = state;

        switch (ledState) {
            case YELLOW_AND_BLUE:
                setAnimation(0, AnimationType.NONE);
                currentColors = new Colors[] { Colors.YELLOW, Colors.BLUE };
                break;
            case PURPLE:
                setAnimation(0, AnimationType.NONE);
                currentColors = new Colors[] { Colors.PURPLE };
                break;
            case YELLOW:
                setAnimation(0, AnimationType.NONE);
                currentColors = new Colors[] { Colors.YELLOW };
                break;
            case OFF:
                setAnimation(0, AnimationType.NONE);
                currentColors = new Colors[] { Colors.BLANK };
                break;
            case YELLOW_AND_BLUE_ANIMATION:
                setAnimation(25, AnimationType.SCROLL);
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
            if (animationType == AnimationType.NONE) {
                fillLEDs(currentColors);
            } else {
                currentAnimationTime = (currentAnimationTime + 1) % animationSpeed;

                if (currentAnimationTime == 0) {
                    switch (animationType) {
                        case SCROLL:
                            scrollAnimateLEDs(currentColors);
                            break;
                        default:
                            break;
                    }
                }
            }
        }

        ledStrip.setData(ledBuffer);
    }
}