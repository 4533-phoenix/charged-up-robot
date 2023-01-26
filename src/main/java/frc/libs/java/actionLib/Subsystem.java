package frc.libs.java.actionLib;

import java.util.concurrent.locks.ReentrantLock;

public class Subsystem {
    private ReentrantLock subsystemThreadLock = new ReentrantLock(true);

    private final Action periodicAction = new Action(
        () -> {},
        () -> { this.periodic(); },
        () -> {}, 
        false
    ).withSubsystem(this);

    private final Action loggingAction = new Action(
        () -> {},
        () -> { this.log(); },
        () -> {}, 
        false
    ).withSubsystem(this);

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
