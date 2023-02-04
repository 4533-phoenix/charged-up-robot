package frc.libs.java.actions.auto;

import frc.libs.java.actions.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Auto;

public class WaitAction extends Action {
    private final double mTime;
    private double mStartTime;

    public WaitAction(double time) {
        super(Auto.getEmptyRunnable(), Auto.getEmptyRunnable(), Auto.getEmptyRunnable(), true);

        this.mTime = time;
    }

    @Override
    public void runStart() {
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void run() {}

    @Override
    public void runEnd() {}

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime >= mTime;
    }
}
