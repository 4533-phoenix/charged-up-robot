import frc.libs.java.actionLib.*;

import java.util.concurrent.Callable;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

public final class ActionRunnerTest {
    private ActionRunner runner = new ActionRunner();

    private int a = 0;
    private int b = 0;
    private int c = 0;
    private int d = 0;

    @Test
    public void runActionRunnerWithActionsWithLocksThatWillCancel() {
        Runnable aRunnable = new Runnable() {
            @Override
            public void run() {
                a += 5;
            }
        };

        Runnable bRunnable = new Runnable() {
            @Override
            public void run() {
                b += a + 10;
            }
        };

        Runnable cRunnable = new Runnable() {
            @Override
            public void run() {
                c += 10;
            }
        };

        Runnable dRunnable = new Runnable() {
            @Override
            public void run() {
                d += c + 20;
            }
        };

        Subsystem abLock = new Subsystem();
        Subsystem cdLock = new Subsystem();

        Action aThread = new Action(() -> {}, aRunnable, () -> {}, true).withSubsystem(abLock);
        Action bThread = new Action(() -> {}, bRunnable, () -> {}, true).withSubsystem(abLock);
        Action cThread = new Action(() -> {}, cRunnable, () -> {}, true).withSubsystem(cdLock);
        Action dThread = new Action(() -> {}, dRunnable, () -> {}, true).withSubsystem(cdLock);

        runner.add(
            aThread,
            bThread,
            cThread,
            dThread
        );

        runner.enable();

        runner.run();

        while (aThread.isAlive() || bThread.isAlive() || cThread.isAlive() || dThread.isAlive()) {}

        runner.disable();

        assertEquals(5, a, 0);
        assertEquals(15, b, 0);
        assertEquals(10, c, 0);
        assertEquals(30, d, 0);
    }

    @Test
    public void runActionRunnerWithFunctionalActionsWithLocksThatWillCancel() {
        Callable<Boolean> aConditionalMethod = new Callable<Boolean>() {
            @Override
            public Boolean call() {
                return a >= 15 && b <= -20;
            }
        };

        Runnable aRunMethod = new Runnable() {
            @Override
            public void run() {
                a += 3;
                b -= 5;
            }
        };

        Callable<Boolean> bConditionalMethod = new Callable<Boolean>() {
            @Override
            public Boolean call() {
                return c >= 20 && d <= -15;
            }
        };

        Runnable bRunMethod = new Runnable() {
            @Override
            public void run() {
                c += (a - 5);
                d += (b + 20);
            }
        };

        Subsystem abLock = new Subsystem();

        FunctionalAction aThread = new FunctionalAction(() -> {}, aRunMethod, aConditionalMethod, () -> {}).withSubsystem(abLock);
        FunctionalAction bThread = new FunctionalAction(() -> {}, bRunMethod, bConditionalMethod, () -> {}).withSubsystem(abLock);

        runner.add(
            aThread,
            bThread
        );

        runner.enable();

        runner.run();

        while (aThread.isAlive() || bThread.isAlive()) {}

        runner.disable();

        assertEquals(a, 15, 0);
        assertEquals(b, -25, 0);
        assertEquals(c, 30, 0);
        assertEquals(d, -15, 0);
    }
}
