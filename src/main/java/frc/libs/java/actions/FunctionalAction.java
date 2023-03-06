package frc.libs.java.actions;

import java.util.concurrent.Callable;

import frc.robot.Constants.ActionConstants;

public final class FunctionalAction extends Action {
    private Callable<Boolean> conditionMethod;

    public FunctionalAction(Runnable startMethod, Runnable runMethod, Callable<Boolean> conditionMethod, Runnable endMethod) {
        super(startMethod, runMethod, endMethod, ActionConstants.WILL_CANCEL);

        this.conditionMethod = conditionMethod;
    }

    @Override
    public FunctionalAction withSubsystem(Subsystem... subsystems) {
        for (Subsystem s : subsystems) {
            super.withSubsystem(s);
        }

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
