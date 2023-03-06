package frc.libs.java.actions;

import java.util.ArrayList;

public final class ActionRunner {
    private ArrayList<Action> actions = new ArrayList<Action>();

    private boolean isEnabled = false;

    public ActionRunner() {}

    public void enable() {
        this.isEnabled = true;

        for (Action action : this.actions) {
            action.runStart();
        }
    }

    public void run() {
        if (!isEnabled) {
            return;
        }

        for (int i = 0; i < this.actions.size(); i++) {
            Action action = this.actions.get(i);

            if (action.willThreadRun() && !action.hasStarted()) {
                action.setStarted();

                action.start();

                while (!action.getThreadLock().isLocked()) {}
            }
            else if (!action.willThreadRun()) {
                action.run();
            }

            if (action.willThreadRun() && action.isFinished()) {
                action.runEnd();

                this.actions.remove(i);

                i--;
            }
        }
    }

    public void disable() {
        this.isEnabled = false;

        for (Action action : this.actions) {
            action.runEnd();
        }
    }

    public void add(Action... actions) {
        for (Action action : actions) {
            this.actions.add(action);
        }
    }
}
