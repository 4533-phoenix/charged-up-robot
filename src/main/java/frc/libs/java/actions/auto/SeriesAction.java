package frc.libs.java.actions.auto;

import frc.libs.java.actions.*;
import frc.robot.Constants.ActionConstants;

import java.util.*;

public final class SeriesAction extends Action {
    private final ArrayList<Action> mRemainingActions = new ArrayList<Action>();

    public SeriesAction(Action... mActions) {
        super(() -> {}, () -> {}, () -> {}, ActionConstants.WILL_CANCEL);

        this.mRemainingActions.addAll(Arrays.asList(mActions));
    }

    @Override
    public void run() {
        if (this.willThreadRun()) {
            this.getThreadLock().lock();
        }

        this.mRemainingActions.forEach(Action::runStart);

        for (Action action : mRemainingActions) {
            if (action.willThreadRun() && !action.hasStarted()) {
                action.withSubsystem(new Subsystem());

                action.setStarted();

                action.start();

                while (!action.isFinished()) {}
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
