package frc.libs.java.actionLib;

import edu.wpi.first.wpilibj.Joystick;



public final class Controller extends Joystick {
    private final ControllerButton[] buttons = new ControllerButton[10];

    private final Action updateButtonsAction = new Action(
        () -> {},
        () -> { this.updateButtons(); },
        () -> {},
        false
    );

    public Controller(int controllerPort) {
        super(controllerPort);

        for (int i = 0; i < this.buttons.length; i++) {
            this.buttons[i] = new ControllerButton();
        }
    }

    public void addButtonEventThread(int buttonID, Listener eventThread) {
        this.buttons[buttonID].addEventThread(eventThread);
    }

    private void pressButton(int buttonID) {
        this.buttons[buttonID].whenPressed();
    }

    private void unpressButton(int buttonID) {
        this.buttons[buttonID].whenReleased();
    }

    private void updateButtons() {
        for (int buttonID = 0; buttonID < buttons.length; buttonID++) {
            // Add by one in order to line up correctly with the Drive Station controller button ID's
            if (this.getRawButton(buttonID + 1)) { 
                this.pressButton(buttonID);
            }
            else {
                this.unpressButton(buttonID);
            }
        }
    }

    public Action getUpdateButtonsAction() {
        return this.updateButtonsAction;
    }
}
