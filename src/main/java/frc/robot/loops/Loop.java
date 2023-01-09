package frc.robot.loops;

// Credit: Team 1678

public interface Loop {
    public void onStart(double timestamp);

    public void onLoop(double timestamp);

    public void onStop(double timestamp);
}
