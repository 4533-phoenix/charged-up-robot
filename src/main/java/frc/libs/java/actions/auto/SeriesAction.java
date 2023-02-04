package frc.libs.java.actions.auto;

import frc.libs.java.actions.*;
import java.util.*;
import frc.robot.subsystems.Auto;

public class SeriesAction extends Action {
    private final ArrayList<Action> mRemainingActions;
    private Action mCurrentAction;

    public SeriesAction(ArrayList<Action> mActions) {
        super(Auto.getEmptyRunnable(), Auto.getEmptyRunnable(), Auto.getEmptyRunnable(), true);

        this.mRemainingActions = mActions;
    }

    public SeriesAction(Action... mActions) {
        super(Auto.getEmptyRunnable(), Auto.getEmptyRunnable(), Auto.getEmptyRunnable(), true);

        this.mRemainingActions = (ArrayList<Action>) Arrays.asList(mActions);
    }

    @Override
    public void runStart() {}

    @Override
    public void run() {
        if (mCurrentAction == null) {
            if (mRemainingActions.isEmpty()) {
                return;
            }

            mCurrentAction = mRemainingActions.remove(0);
            mCurrentAction.start();
        }

        mCurrentAction.run();

        if (mCurrentAction.isFinished()) {
            mCurrentAction.runEnd();
            mCurrentAction = null;
        }
    }

    @Override
    public void runEnd() {}

    @Override
    public boolean isFinished() {
        return mRemainingActions.isEmpty() && mCurrentAction == null;
    }
}
