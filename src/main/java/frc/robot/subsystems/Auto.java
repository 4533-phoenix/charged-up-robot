package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.auto.SwervePath;
import frc.robot.auto.SwervePath.PathState;
import frc.robot.loops.*;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.Timer;

import java.io.File;
import java.util.Scanner;
import java.util.ArrayList;
import java.util.Arrays;

public final class Auto extends Subsystem {
    private static Auto mInstance;

    private boolean isEnabled = false;

    private HolonomicDriveController autoController = new HolonomicDriveController(
        new PIDController(
            AutoConstants.AUTO_X_VELOCITY_KP,
            AutoConstants.AUTO_X_VELOCITY_KI,
            AutoConstants.AUTO_X_VELOCITY_KD
        ),
        new PIDController(
            AutoConstants.AUTO_Y_VELOCITY_KP,
            AutoConstants.AUTO_Y_VELOCITY_KI,
            AutoConstants.AUTO_Y_VELOCITY_KD
        ),
        new ProfiledPIDController(
            AutoConstants.AUTO_OMEGA_KP,
            AutoConstants.AUTO_OMEGA_KI,
            AutoConstants.AUTO_OMEGA_KD,
            AutoConstants.AUTO_OMEGA_CONSTRAINTS
        )
    );

    public static Auto getInstance() {
        if (mInstance == null) {
            mInstance = new Auto();
        }

        return mInstance;
    }

    public Auto() {}

    public void enable() {
        this.isEnabled = true;
    }

    public void disable() {
        this.isEnabled = false;
    }

    public boolean isEnabled() {
        return this.isEnabled;
    }

    public HolonomicDriveController getAutoController() {
        return this.autoController;
    }

    private static final class AutoLoops {
        public Loop testAutonomous() {
            return new Loop() {
                private Trajectory testAutonomousTrajectory;
                
                private boolean hasStarted = false;
                private double startTime;

                @Override
                public void onStart(double timestamp) {
                    TrajectoryConfig config = new TrajectoryConfig(
                        DriveConstants.DRIVE_MAX_VELOCITY, 
                        DriveConstants.DRIVE_MAX_ACCELERATION
                    )
                    .addConstraint(
                        new MaxVelocityConstraint(
                            DriveConstants.DRIVE_MAX_VELOCITY
                        )
                    )
                    .addConstraint(
                        new SwerveDriveKinematicsConstraint(
                            DriveConstants.SWERVE_KINEMATICS, 
                            DriveConstants.DRIVE_MAX_VELOCITY
                        )
                    );

                    Pose2d startPose = Swerve.getInstance().getSwervePose();

                    ArrayList<Translation2d> trajectoryPoints = new ArrayList<Translation2d>(
                        Arrays.asList(
                            new Translation2d(startPose.getX() + 0.25, startPose.getY() + 0.25),
                            new Translation2d(startPose.getX() + 0.50, startPose.getY() - 0.25)
                        )
                    );

                    Pose2d endPose = new Pose2d(startPose.getX() + 0.75 , startPose.getY(), startPose.getRotation());

                    this.testAutonomousTrajectory = TrajectoryGenerator.generateTrajectory(
                        startPose, 
                        trajectoryPoints, 
                        endPose, 
                        config
                    );
                }

                @Override
                public void onLoop(double timestamp) {
                    if (Auto.getInstance().isEnabled()) {
                        double currTime = Timer.getFPGATimestamp();

                        if (!this.hasStarted) {
                            this.startTime = currTime;

                            this.hasStarted = true;
                        }

                        if (currTime - this.startTime > this.testAutonomousTrajectory.getTotalTimeSeconds()) {
                            SwerveModuleState[] swerveModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
                                new ChassisSpeeds(0.0, 0.0, 0.0)
                            );

                            Swerve.getInstance().setModuleStates(swerveModuleStates);

                            return;
                        }

                        HolonomicDriveController autoController = Auto.getInstance().getAutoController();
                        
                        Trajectory.State trajectoryState = this.testAutonomousTrajectory.sample(currTime - this.startTime);

                        ChassisSpeeds chassisSpeeds = autoController.calculate(
                            Swerve.getInstance().getSwervePose(),
                            trajectoryState,
                            trajectoryState.poseMeters.getRotation()
                        );

                        SwerveModuleState[] swerveModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

                        Swerve.getInstance().setModuleStates(swerveModuleStates);
                    }
                }

                @Override
                public void onStop(double timestamp) {}
            };
        }

        public Loop testPathplannerAutonomous() {
            return new Loop() {
                private SwervePath testAuto;
                
                private boolean hasStarted = false;
                private double startTime;

                @Override
                public void onStart(double timestamp) {
                    this.testAuto = SwervePath.fromCSV("Test Path");
                }

                @Override
                public void onLoop(double timestamp) {
                    if (Auto.getInstance().isEnabled()) {
                        double currTime = Timer.getFPGATimestamp();

                        if (!hasStarted) {
                            this.startTime = currTime;

                            this.hasStarted = true;
                        }

                        double time = currTime - startTime;

                        PathState currState = testAuto.sample(time);

                        ChassisSpeeds chassisSpeeds = Auto.getInstance().getAutoController().calculate(
                            Swerve.getInstance().getSwervePose(),
                            currState.getTrajectoryState(),
                            currState.rotation
                        );

                        SwerveModuleState[] swerveModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

                        Swerve.getInstance().setModuleStates(swerveModuleStates);
                    }
                }

                @Override
                public void onStop(double timestamp) {}
            };
        }
    }

    @Override
    public void registerEnabledLoops(ILooper looper) {
        AutoLoops autoLoops = new AutoLoops();

        looper.register(autoLoops.testAutonomous());
        // looper.register(autoLoops.testPathplannerAutonomous());
    }

    @Override
    public void stop() {
        // TODO: Add code that will fully stop this subsystem
    }

    @Override
    public boolean checkSystem() {
        // TODO: Add code that checks possible system faults (should be SERIOUS FAULTS)
        // Serious faults should be things that could damage or hurt other people/things
        // Serious faults due to the fact that stop() will most likely be called
        // after this returning false
        return true;
    }
}