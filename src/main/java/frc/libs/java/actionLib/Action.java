package frc.libs.java.actionLib;

import java.util.concurrent.locks.ReentrantLock;

public class Action extends Thread implements Listener {
    private Runnable startMethod;

    private Runnable runMethod;

    private Runnable endMethod;

    private boolean willCancel;

    private boolean willThreadRun = false;
    
    private boolean willRun = true;

    private ReentrantLock threadLock;

    public Action(Runnable startMethod, Runnable runMethod, Runnable endMethod, boolean willCancel) {
        super(runMethod);

        this.startMethod = startMethod;

        this.runMethod = runMethod;

        this.endMethod = endMethod;

        this.willCancel = willCancel;

        if (this.willCancel) {
            this.willThreadRun = true;
        }
    }

    public Action(Runnable startMethod, Runnable runMethod, Runnable endMethod, Controller controller, int buttonID) {
        super(runMethod);

        this.startMethod = startMethod;

        this.runMethod = runMethod;

        this.endMethod = endMethod;

        this.willCancel = false;

        this.willThreadRun = true;
        
        this.willRun = false;

        controller.addButtonEventThread(buttonID, this);
    }

    public Action withSubsystem(Subsystem subsystem) {
        this.threadLock = subsystem.getSubsystemThreadLock();

        return this;
    }

    public void runStart() {
        this.startMethod.run();
    }

    public void run() {
        if (!this.willRun()) {
            return;
        }
        
        if (this.willThreadRun()) {
            try {
                this.getThreadLock().lock();
            }
            finally {}
        }

        runMethod.run();

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

    @Override
    public void respondTrue() {
        this.willRun = true;
    }

    @Override
    public void respondFalse() {
        this.willRun = false;
    }

    public boolean willCancel() {
        return this.willCancel;
    }

    public boolean willThreadRun() {
        return this.willThreadRun;
    }

    public boolean willRun() {
        return this.willRun;
    }

    public ReentrantLock getThreadLock() {
        return this.threadLock;
    }
}
