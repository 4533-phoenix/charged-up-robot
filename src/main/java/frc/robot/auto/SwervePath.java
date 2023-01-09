package frc.robot.auto;

// Credit: Team 3015

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.GlobalConstants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;

import java.io.*;
import java.util.*;

public class SwervePath {
    private ArrayList<PathState> states;

    /**
     * Construct a new Swerve Path
     */
    public SwervePath() {
        this.states = new ArrayList<>();
    }

    /**
     * Get all of the states in the path
     *
     * @return An arraylist of all path states
     */
    public ArrayList<PathState> getStates() {
        return this.states;
    }

    /**
     * Get the number of states in the path
     *
     * @return The number of states
     */
    public int numStates() {
        return this.states.size();
    }

    /**
     * Get the total runtime of a path
     *
     * @return Total runtime in seconds
     */
    public double getRuntime() {
        return states.get(states.size() - 1).time;
    }

    /**
     * Get the initial state in the path
     *
     * @return First state in the path
     */
    public PathState getInitialState() {
        return this.states.get(0);
    }

     /**
     * Create a SwervePath object from a CSV file
     * Expected format is xPos, yPos, velocity, acceleration, heading (direction robot is moving), rotation
     *
     * @param filename The path file to load
     * @return The SwervePath object
     */
    public static SwervePath fromCSV(String filename) {
        SwervePath traj = new SwervePath();

        try (BufferedReader br = new BufferedReader(new FileReader(new File(Filesystem.getDeployDirectory(), "paths/" + filename + ".csv")))) {
            String line = "";
            
            br.readLine();
            br.readLine();
            line = br.readLine();

            while (line != null) {
                String[] point = line.split(",");

                double time = Double.parseDouble(point[0]);
                double xPos = Double.parseDouble(point[1]);
                double yPos = Double.parseDouble(point[2]);
                double heading = Double.parseDouble(point[3]);
                double vel = Double.parseDouble(point[4]);
                double acc = Double.parseDouble(point[5]);
                double curv = Double.parseDouble(point[6]);
                double rot = Double.parseDouble(point[7]);
                double angularVel = Double.parseDouble(point[8]);
                double holonomicAngularVel = Double.parseDouble(point[9]);

                traj.states.add(new PathState(new Translation2d(xPos, yPos), Rotation2d.fromDegrees(heading), vel, angularVel, acc, Rotation2d.fromDegrees(rot), time, curv));

                line = br.readLine();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return traj;
    }

    private static double lerp(double startVal, double endVal, double t) {
        return startVal + (endVal - startVal) * t;
    }

    private static Translation2d lerp(Translation2d startVal, Translation2d endVal, double t) {
        return startVal.plus((endVal.minus(startVal)).times(t));
    }

    private static Rotation2d lerp(Rotation2d startVal, Rotation2d endVal, double t) {
        return startVal.plus((endVal.minus(startVal)).times(t));
    }

    /**
     * Sample the path at a given point in time
     *
     * @param time The elapsed time to sample
     * @return State of the path at the given time
     */
    public PathState sample(double time) {
        if (time <= states.get(0).time) {
            return states.get(0);
        }

        if (time >= getRuntime()) {
            return states.get(states.size() - 1);
        }

        int low = 1;
        int high = states.size() - 1;

        while (low != high) {
            int mid = (low + high) / 2;
            if (states.get(mid).time < time) {
                low = mid + 1;
            } else {
                high = mid;
            }
        }

        PathState sample = states.get(low);
        PathState prevSample = states.get(low - 1);

        if (Math.abs(sample.time - prevSample.time) < 1E-5) {
            return sample;
        }

        return prevSample.interpolate(sample, (time - prevSample.time) / (sample.time - prevSample.time));
    }

    public static class PathState {
        private final Translation2d pos;
        private final Rotation2d heading;
        private final double velocity;
        private final double angularVelocity;
        private final double acceleration;
        public Rotation2d rotation;
        private final double time;
        private final double curvature;

        /**
         * Construct a State
         *
         * @param pos               Translation2d holding the robot's position on the field
         * @param heading           Rotation2d representing the direction of robot motion
         * @param velocity          Velocity of the robot
         * @param angularVelocity   Angular velocity of the robot 
         * @param acceleration      Acceleration of the robot
         * @param rotation          Rotation of the robot
         * @param time              Time this state represents
         * @param curvature         Curvature of the path
         */
        public PathState(Translation2d pos, Rotation2d heading, double velocity, double angularVelocity, double acceleration, Rotation2d rotation, double time, double curvature) {
            this.pos = pos;
            this.heading = heading;
            this.velocity = velocity;
            this.angularVelocity = angularVelocity;
            this.acceleration = acceleration;
            this.rotation = rotation;
            this.time = time;
            this.curvature = curvature;
        }

        public Translation2d getPos() {
            return this.pos;
        }

        public double getVelocity() {
            return this.velocity;
        }

        public Rotation2d getRotation() {
            return this.rotation;
        }

        public Pose2d getPose() {
            return new Pose2d(this.pos, this.rotation);
        }

        public Rotation2d getHeading(){
            return this.heading;
        }

        public double getTime() {
            return this.time;
        }

        public State getTrajectoryState() {
            return new State(this.time, this.velocity, this.acceleration, this.getPose(), this.curvature);
        }

        PathState interpolate(PathState endVal, double t) {
            double newT = lerp(time, endVal.time, t);
            double deltaT = newT - time;

            if (deltaT < 0) {
                return endVal.interpolate(this, 1 - t);
            }

            double newV = velocity + (acceleration * deltaT);

            double interpolationFrac = newT / deltaT;

            return new PathState(
                    lerp(pos, endVal.pos, interpolationFrac),
                    lerp(heading, endVal.heading, interpolationFrac),
                    newV,
                    acceleration,
                    angularVelocity,
                    lerp(rotation, endVal.rotation, interpolationFrac),
                    newT,
                    curvature
            );
        }
    }
}
