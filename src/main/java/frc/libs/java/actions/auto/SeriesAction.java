package frc.libs.java.actions.auto;

import frc.libs.java.actions.*;
import frc.robot.Constants.ActionConstants;

import java.util.*;

public final class SeriesAction extends Action {
    private final ArrayList<Action> mRemainingActions;

    public SeriesAction(Action... mActions) {
        super(() -> {}, () -> {}, () -> {}, ActionConstants.WILL_CANCEL);

        this.mRemainingActions = (ArrayList<Action>) Arrays.asList(mActions);
    }

    @Override
    public void run() {
        if (this.willThreadRun()) {
            this.getThreadLock().lock();
        }

        this.mRemainingActions.forEach(Action::runStart);

        for (Action action : mRemainingActions) {
            if (action.willThreadRun() && action.getState() == State.NEW) {
                action.withSubsystem(new Subsystem());

                action.start();

                while (!action.getThreadLock().isLocked() || action.isAlive()) {}
            }
            else {
                action.run();
            }
        }

        this.mRemainingActions.forEach(Action::runEnd);

        if (this.willThreadRun()) {
            this.getThreadLock().unlock();
        }
    }
}
