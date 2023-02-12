package frc.libs.java.actions;

import java.util.concurrent.locks.ReentrantLock;

import frc.robot.Constants.ActionConstants;

public class Subsystem {
    private ReentrantLock subsystemThreadLock = new ReentrantLock(true);

    private final Action periodicAction = new Action(
        () -> {},
        () -> { this.periodic(); },
        () -> {}, 
        ActionConstants.WILL_NOT_CANCEL
    );

    private final Action loggingAction = new Action(
        () -> {},
        () -> { this.log(); },
        () -> {}, 
        ActionConstants.WILL_NOT_CANCEL
    );

    public Subsystem() {}

    public ReentrantLock getSubsystemThreadLock() {
        return this.subsystemThreadLock;
    }

    protected final Action getPeriodicAction() {
        return this.periodicAction;
    }

    protected final Action getLoggingAction() {
        return this.loggingAction;
    }

    public void log() {}

    public void periodic() {}

    public void queryInitialActions() {}
}
