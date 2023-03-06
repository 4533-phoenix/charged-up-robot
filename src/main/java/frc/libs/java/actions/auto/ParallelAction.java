package frc.libs.java.actions.auto;

import frc.libs.java.actions.*;
import frc.robot.Constants.ActionConstants;

import java.util.*;

public final class ParallelAction extends Action {
    private final ArrayList<Action> mActions;

    public ParallelAction(Action... mActions) {
        super(() -> {}, () -> {}, () -> {}, ActionConstants.WILL_CANCEL);

        this.mActions = (ArrayList<Action>) Arrays.asList(mActions);
    }

    @Override
    public void run() {
        if (this.willThreadRun()) {
            this.getThreadLock().lock();
        }
        
        this.mActions.forEach(Action::runStart);

        for (Action action : this.mActions) {
            if (action.willThreadRun() && !action.hasStarted()) {
                action.withSubsystem(new Subsystem());

                action.setStarted();

                action.start();
            }
            else {
                action.run();
            }
        }

        boolean isFinished = false;

        while (!isFinished) {
            isFinished = true;

            for (Action action : this.mActions) {
                if (action.willThreadRun() && !action.isFinished()) {
                    isFinished = false;

                    break;
                }
            }
        }

        this.mActions.forEach(Action::runEnd);

        if (this.willThreadRun()) {
            this.getThreadLock().unlock();
        }
    }
}
