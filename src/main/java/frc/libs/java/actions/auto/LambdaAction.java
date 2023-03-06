package frc.libs.java.actions.auto;

import frc.libs.java.actions.*;
import frc.robot.Constants.*;

public class LambdaAction extends Action {
    public interface VoidInterface {
        void f();
    }

    private VoidInterface mF;

    public LambdaAction(VoidInterface f) {
        super(() -> {}, () -> {}, () -> {}, ActionConstants.WILL_CANCEL);

        this.mF = f;
    }

    @Override
    public void run() {
        if (this.willThreadRun()) {
            try {
                this.getThreadLock().lock();
            }
            finally {}
        }

        this.mF.f();

        if (this.willThreadRun()) {
            try {
                this.getThreadLock().unlock();
            }
            finally {}
        }
    }
}