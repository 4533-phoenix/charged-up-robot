package frc.libs.java.actions.auto;

import frc.libs.java.actions.*;
import frc.robot.Constants.ActionConstants;
import edu.wpi.first.wpilibj.Timer;

public final class WaitAction extends Action {
    private final double mTime;

    public WaitAction(double time) {
        super(() -> {}, () -> {}, () -> {}, ActionConstants.WILL_CANCEL);

        this.mTime = time;
    }

    @Override
    public void run() {
        if (this.willThreadRun()) {
            try {
                this.getThreadLock().lock();
            }
            finally {}
        }

        double startTime = Timer.getFPGATimestamp();

        while (Timer.getFPGATimestamp() - startTime <= this.mTime) {}

        if (this.willThreadRun()) {
            try {
                this.getThreadLock().unlock();
            }
            finally {}
        }
    }
}
