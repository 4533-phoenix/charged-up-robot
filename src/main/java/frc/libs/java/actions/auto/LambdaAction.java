package frc.libs.java.actions.auto;

import frc.libs.java.actions.*;
import frc.robot.subsystems.Auto;

public class LambdaAction extends Action {
    public interface VoidInterface {
        void f();
    }

    VoidInterface mF;

    public LambdaAction(VoidInterface f) {
        super(Auto.getEmptyRunnable(), Auto.getEmptyRunnable(), Auto.getEmptyRunnable(), true);

        this.mF = f;
    }

    @Override
    public void runStart() {
        mF.f();
    }

    @Override
    public void run() {}

    @Override
    public void runEnd() {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
