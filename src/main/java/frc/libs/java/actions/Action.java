package frc.libs.java.actions;

import java.util.concurrent.locks.ReentrantLock;

public class Action extends Thread {
    private Runnable startMethod;

    private Runnable runMethod;

    private Runnable endMethod;

    private boolean willThreadRun;

    private ReentrantLock threadLock = new ReentrantLock();

    private boolean isFinished;

    private boolean hasStarted = false;

    public Action(Runnable startMethod, Runnable runMethod, Runnable endMethod, boolean willCancel) {
        super(runMethod);

        this.startMethod = startMethod;

        this.runMethod = runMethod;

        this.endMethod = endMethod;

        this.willThreadRun = willCancel;
    }

    public Action withSubsystem(Subsystem subsystem) {
        this.threadLock = subsystem.getSubsystemThreadLock();

        return this;
    }

    public void runStart() {
        this.startMethod.run();
    }

    public void run() {    
        if (this.willThreadRun()) {
            try {
                this.getThreadLock().lock();
            }
            finally {}
        }

        runMethod.run();

        this.isFinished = true;

        if (this.willThreadRun()) {
            try {
                this.getThreadLock().unlock();
            }
            finally {}
        }
    }

    public void runEnd() {
        this.endMethod.run();
    }

    public boolean willThreadRun() {
        return this.willThreadRun;
    }

    public boolean isFinished() {
        return this.isFinished;
    }

    public boolean hasStarted() {
        return this.hasStarted;
    }

    public void setStarted() {
        this.hasStarted = true;
    }

    public ReentrantLock getThreadLock() {
        return this.threadLock;
    }
}
