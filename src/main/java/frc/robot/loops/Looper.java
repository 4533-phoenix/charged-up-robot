package frc.robot.loops;

// Credit: Team 1678

import java.util.*;

import edu.wpi.first.wpilibj.Timer;

public final class Looper implements ILooper {
    private boolean running;

    private final List<Loop> loops;
    private double timestamp = 0.0;

    public Looper() {
        this.running = false;
        this.loops = new ArrayList<>();
    }

    @Override
    public void register(Loop loop) {
        this.loops.add(loop);
    }

    public void start() {
        if (!this.running) {
            this.timestamp = Timer.getFPGATimestamp();

            for (Loop loop : loops) {
                loop.onStart(timestamp);
            }

            this.running = true;
        }
    }

    public void loop() {
        if (this.running) {
            this.timestamp = Timer.getFPGATimestamp();

            for (Loop loop : loops) {
                loop.onLoop(timestamp);
            }
        }
    }

    public synchronized void stop() {
        if (this.running) {
            this.running = false;

            this.timestamp = Timer.getFPGATimestamp();

            for (Loop loop : loops) {
                loop.onStop(this.timestamp);
            }
        }
    }
}
