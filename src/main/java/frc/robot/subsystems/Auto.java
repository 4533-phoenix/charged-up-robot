package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.subsystems.Extension.ExtensionState;
import frc.robot.subsystems.Extension.LowerExtensionState;
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
import frc.libs.java.actions.Action;
import frc.libs.java.actions.Subsystem;
import frc.libs.java.actions.auto.DrivePathAction;
import frc.libs.java.actions.auto.SeriesAction;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;

public final class Auto extends Subsystem {
    private static Auto mInstance;

    private static final Map<String, Action> autoCommands = Map.ofEntries(
        Map.entry("Test Autonomous", AutoActions.testAutonomous())
        // Map.entry("Blue Bottom Cube Autonomous", AutoActions.blueBottomCubeAutonomous())
    );

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

    private Auto() {}

    public static Auto getInstance() {
        if (mInstance == null) {
            mInstance = new Auto();
        }

        return mInstance;
    }

    public HolonomicDriveController getAutoController() {
        return this.autoController;
    }

    public SwerveModuleState[] getSwerveModuleStates(Trajectory.State trajectoryState) {
        ChassisSpeeds chassisSpeeds = this.autoController.calculate(
            PoseEstimator.getInstance().getSwervePose(),
            trajectoryState,
            trajectoryState.poseMeters.getRotation()
        );

        return DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    }

    private static final class AutoActions {
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

                while (timer.get() <= 1.5) {               
                    Trajectory.State trajectoryState = testAutonomousTrajectory.sample(timer.get());

                    Swerve.getInstance().setModuleStates(Auto.getInstance().getSwerveModuleStates(trajectoryState));
                }

                System.out.println(testAutonomousTrajectory.getTotalTimeSeconds());

                timer.stop();

                Swerve.getInstance().setModuleStates(DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds()));

                Gripper.getInstance().enableGripper();

                try {
                    Thread.sleep(2500);
                }
                catch (Exception e) {
                    e.printStackTrace();
                }

                timer.start();

                while (timer.get() <= testAutonomousTrajectory.getTotalTimeSeconds()) {
                    Trajectory.State trajectoryState = testAutonomousTrajectory.sample(timer.get());

                    Swerve.getInstance().setModuleStates(Auto.getInstance().getSwerveModuleStates(trajectoryState));
                }

                if (timer.get() > testAutonomousTrajectory.getTotalTimeSeconds()) {
                    Gripper.getInstance().disableGripper();
                }
            };

            Runnable endMethod = () -> {
                Swerve.getInstance().setModuleStates(DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds()));
            };

            return new Action(startMethod, runMethod, endMethod, ActionConstants.WILL_CANCEL).withSubsystem(Auto.getInstance());
        }

        public static final Action blueBottomCubeAutonomous() {
            Action blueBottomCubeRetrieve = new DrivePathAction("Blue Bottom Cube Retrieve");

        //     Action getBlueBottomCube = new Action(
        //         () -> {}, 
        //         () -> {
        //             if (Gripper.getInstance().isDroppingObject()) {
        //                 Gripper.getInstance().enableGripper();
        //             }
        //             else {
        //                 Gripper.getInstance().disableGripper();

        //                 try {
        //                     Thread.sleep(250);
        //                 }
        //                 catch (Exception e) {
        //                     e.printStackTrace();
        //                 }

                        Gripper.getInstance().enableGripper();
                    }
                }, 
                () -> {}, 
                ActionConstants.WILL_CANCEL
            );

            Action blueBottomCubeScore = new DrivePathAction("Blue Bottom Cube Score");

        //     Action scoreBlueBottomCube = new Action(
        //         () -> {}, 
        //         () -> {
        //             Extension.getInstance().setExtensionState(ExtensionState.HIGH_ROW);

        //             try {
        //                 Thread.sleep(1000);
        //             }
        //             catch (Exception e) {
        //                 e.printStackTrace();
        //             }

                    Gripper.getInstance().disableGripper();
                }, 
                () -> {
                    Extension.getInstance().setLowerExtensionState(LowerExtensionState.OFF);
                }, 
                ActionConstants.WILL_CANCEL
            );

        //     return new SeriesAction(
        //         blueBottomCubeRetrieve,
        //         getBlueBottomCube,
        //         blueBottomCubeScore,
        //         scoreBlueBottomCube
        //     ).withSubsystem(Auto.getInstance());
        // }
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

    public Action getAutonomous(String key) {
        return autoCommands.get(key);
    }
}