package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.auto.SwervePath;
import frc.robot.auto.SwervePath.PathState;
import frc.robot.Robot;

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

import frc.libs.java.actionLib.Action;
import frc.libs.java.actionLib.Subsystem;

import java.util.ArrayList;
import java.util.Arrays;

public final class Auto extends Subsystem {
    private static Auto mInstance;

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

    public HolonomicDriveController getAutoController() {
        return this.autoController;
    }

    public static final class AutoActions {
        public static final Action testAutonomous() {
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

            Pose2d startPose = PoseEstimator.getInstance().getSwervePose();

            ArrayList<Translation2d> trajectoryPoints = new ArrayList<Translation2d>(
                Arrays.asList(
                    new Translation2d(startPose.getX() + 0.25, startPose.getY() + 0.25),
                    new Translation2d(startPose.getX() + 0.50, startPose.getY() - 0.25)
                )
            );

            Pose2d endPose = new Pose2d(startPose.getX() + 0.75 , startPose.getY(), startPose.getRotation());

            Trajectory testAutonomousTrajectory = TrajectoryGenerator.generateTrajectory(
                startPose, 
                trajectoryPoints, 
                endPose, 
                config
            );

            Runnable startMethod = () -> {};

            Runnable runMethod = () -> {
                Timer timer = new Timer();
                timer.reset();
                timer.start();

                while (timer.get() <= testAutonomousTrajectory.getTotalTimeSeconds()) {
                    HolonomicDriveController autoController = Auto.getInstance().getAutoController();
                    
                    Trajectory.State trajectoryState = testAutonomousTrajectory.sample(timer.get());

                    ChassisSpeeds chassisSpeeds = autoController.calculate(
                        PoseEstimator.getInstance().getSwervePose(),
                        trajectoryState,
                        trajectoryState.poseMeters.getRotation()
                    );

                    SwerveModuleState[] swerveModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

                    Swerve.getInstance().setModuleStates(swerveModuleStates);
                }
            };

            Runnable endMethod = () -> {
                Swerve.getInstance().setModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
                });
            };

            return new Action(startMethod, runMethod, endMethod, true);
        }

        public static final Action testPathplannerAutonomous() {
            SwervePath testAuto = SwervePath.fromCSV("Test Path");

            Runnable startMethod = () -> {};

            Runnable runMethod = () -> {
                Timer timer = new Timer();
                timer.reset();
                timer.start();

                while (timer.get() <= testAuto.getRuntime()) {
                    PathState currState = testAuto.sample(timer.get());

                    ChassisSpeeds chassisSpeeds = Auto.getInstance().getAutoController().calculate(
                        PoseEstimator.getInstance().getSwervePose(),
                        currState.getTrajectoryState(),
                        currState.rotation
                    );

                    SwerveModuleState[] swerveModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

                    Swerve.getInstance().setModuleStates(swerveModuleStates);
                }
            };

            Runnable endMethod = () -> {
                Swerve.getInstance().setModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
                });
            };

            return new Action(startMethod, runMethod, endMethod, true);
        }
    }

    @Override
    public void log() {}

    @Override
    public void periodic() {}

    @Override
    public void queryInitialActions() {
        Robot.autonomousRunner.add(
            this.getLoggingAction(),
            this.getPeriodicAction() 
        );
    }
}