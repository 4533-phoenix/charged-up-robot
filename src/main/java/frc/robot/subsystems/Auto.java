package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.Robot;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import frc.libs.java.actions.Action;
import frc.libs.java.actions.Subsystem;
import frc.libs.java.actions.auto.DrivePathAction;
import frc.libs.java.actions.auto.LambdaAction;
import frc.libs.java.actions.auto.SeriesAction;
import frc.robot.subsystems.Extension.ExtensionState;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;

public final class Auto extends Subsystem {
    private static Auto mInstance;

    private static final Map<String, Action> autoCommands = Map.ofEntries(
        Map.entry("Test Autonomous", AutoActions.testAutonomous()),
        Map.entry("Blue Bottom Cube Autonomous", AutoActions.blueBottomCubeAutonomous())
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
            Pose2d startPose = new Pose2d(new Translation2d(), Swerve.getInstance().getGyroRotation());

            ArrayList<Pose2d> trajectoryPoints = new ArrayList<Pose2d>(
                Arrays.asList(
                    startPose,
                    new Pose2d(startPose.getX() + 2.0, startPose.getY() - 1.0, startPose.getRotation()),
                    new Pose2d(startPose.getX() + 4.0, startPose.getY(), startPose.getRotation())
                )
            );

            Action driveTestPathAction = new DrivePathAction(trajectoryPoints);

            Action armToMidAction = new LambdaAction(() -> Extension.getInstance().updateExtensionState(ExtensionState.MIDDLE_ROW));

            Action testAuto = new SeriesAction(driveTestPathAction, armToMidAction);

            return driveTestPathAction.withSubsystem(Auto.getInstance());
        }

        public static final Action blueBottomCubeAutonomous() {
            ArrayList<Pose2d> retrievePoints = new ArrayList<Pose2d>(
                Arrays.asList(
                    new Pose2d(1.92, 0.46, new Rotation2d()),
                    new Pose2d(6.49, 0.91, new Rotation2d())
                )
            );

            Action cubeRetrievePath = new DrivePathAction(retrievePoints);

            Action getBlueBottomCube = new LambdaAction(() -> Gripper.getInstance().enableGripper());

            ArrayList<Pose2d> scorePoints = new ArrayList<Pose2d>(
                Arrays.asList(
                    new Pose2d(6.49, 0.91, new Rotation2d()),
                    new Pose2d(2.71, 0.91, Rotation2d.fromDegrees(180.0)),
                    new Pose2d(1.80, 2.69, Rotation2d.fromDegrees(180.0))
                )
            );

            Action cubeScorePath = new DrivePathAction(scorePoints);

            // Action scoreBlueBottomCube = new LambdaAction(() -> Gripper.getInstance().dropObject(Timer.getFPGATimestamp()));

            return new SeriesAction(
                cubeRetrievePath,
                getBlueBottomCube,
                cubeScorePath
                // scoreBlueBottomCube
            ).withSubsystem(Auto.getInstance());
        }
    }

    @Override
    public void log() {}

    @Override
    public void periodic() {
        Extension.getInstance().updateExtensionState();
        Extension.getInstance().updateElbowController();
    }

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