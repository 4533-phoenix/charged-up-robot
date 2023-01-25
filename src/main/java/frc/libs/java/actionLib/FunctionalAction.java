package frc.libs.java.actionLib;

import java.util.concurrent.Callable;

public final class FunctionalAction extends Action {
    private Callable<Boolean> conditionMethod;

    public FunctionalAction(Runnable startMethod, Runnable runMethod, Callable<Boolean> conditionMethod, Runnable endMethod, boolean willCancel) {
        super(startMethod, runMethod, endMethod, willCancel);

        this.conditionMethod = conditionMethod;
    }

    public FunctionalAction(Runnable startMethod, Runnable runMethod, Callable<Boolean> conditionMethod, Runnable endMethod, Controller controller, int buttonID) {
        super(startMethod, runMethod, endMethod, controller, buttonID);

        this.conditionMethod = conditionMethod;
    }

    @Override
    public FunctionalAction withSubsystem(Subsystem subsystem) {
        super.withSubsystem(subsystem);

        return this;
    }

    @Override
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

        try {
            while (!conditionMethod.call()) {
                super.run();
            }
        }
        catch (Exception e) {}
        
        if (this.willThreadRun()) {
            try {
                this.getThreadLock().unlock();
            }
            finally {}
        }
    }
}
