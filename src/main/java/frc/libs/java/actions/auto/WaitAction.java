package frc.libs.java.actions.auto;

import frc.libs.java.actions.*;
import edu.wpi.first.wpilibj.Timer;

public final class WaitAction extends Action {
    private final double mTime;

    public WaitAction(double time) {
        super(() -> {}, () -> {}, () -> {}, true);

        this.mTime = time;
    }

    @Override
    public void run() {
        double startTime = Timer.getFPGATimestamp();

        while (Timer.getFPGATimestamp() - startTime <= this.mTime) {}
    }
}
