package frc.libs.java.actions.auto;

import frc.libs.java.actions.*;
import java.util.*;
import frc.robot.subsystems.Auto;

public class ParallelAction extends Action {
    private final ArrayList<Action> mActions;

    public ParallelAction(ArrayList<Action> mActions) {
        super(Auto.getEmptyRunnable(), Auto.getEmptyRunnable(), Auto.getEmptyRunnable(), true);

        this.mActions = mActions;
    }

    public ParallelAction(Action... mActions) {
        super(Auto.getEmptyRunnable(), Auto.getEmptyRunnable(), Auto.getEmptyRunnable(), true);

        this.mActions = (ArrayList<Action>) Arrays.asList(mActions);
    }

    @Override
    public void runStart() {
        mActions.forEach(Action::runStart);
    }

    @Override
    public void run() {
        mActions.forEach(Action::run);
    }

    @Override
    public void runEnd() {
        mActions.forEach(Action::runEnd);
    }

    @Override
    public boolean isFinished() {
        for (Action action : mActions) {
            if (!action.isFinished()) {
                return false;
            }
        }
        return true;
    }
}
