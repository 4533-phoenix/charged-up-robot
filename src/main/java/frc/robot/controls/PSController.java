package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.*;

public class PSController {
    private final XboxController mController;

    public enum Side {
        LEFT, RIGHT
    }

    public enum Axis {
        X, Y
    }

    public enum Button {
        A(1), B(2), X(3), Y(4), LB(5), RB(6), 
        BACK(7), START(8), L_JOYSTICK(9), R_JOYSTICK(10);

        public final int id;

        Button(int id) {
            this.id = id;
        }
    }

    PSController(int port) {
        mController = new XboxController(port);
    }

    public double getAxis(Side side, Axis axis) {
        boolean left = side == Side.LEFT;
        boolean y = axis == Axis.Y;
        return mController.getRawAxis((left ? 0 : 4) + (y ? 1 : 0));
    }

    public boolean getTrigger(Side side) {
        return mController.getRawAxis(side == Side.LEFT ? 2 : 3) > OIConstants.TRIGGER_THRESHOLD;
    }

    public boolean getButton(Button button) {
        return mController.getRawButton(button.id);
    }

    public XboxController getController() {
        return mController;
    }
}
