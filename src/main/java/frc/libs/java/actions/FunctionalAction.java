package frc.libs.java.actions;

import java.util.concurrent.Callable;

public final class FunctionalAction extends Action {
    private Callable<Boolean> conditionMethod;

    public FunctionalAction(Runnable startMethod, Runnable runMethod, Callable<Boolean> conditionMethod, Runnable endMethod) {
        super(startMethod, runMethod, endMethod, true);

        this.conditionMethod = conditionMethod;
    }

    @Override
    public FunctionalAction withSubsystem(Subsystem subsystem) {
        super.withSubsystem(subsystem);

        return this;
    }

    @Override
    public void run() {
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
