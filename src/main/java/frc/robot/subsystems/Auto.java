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
            Pose2d startPose = PoseEstimator.getInstance().getSwervePose();

            ArrayList<Pose2d> trajectoryPoints = new ArrayList<Pose2d>(
                Arrays.asList(
                    startPose,
                    new Pose2d(startPose.getX() + 0.25, startPose.getY() + 0.25, Rotation2d.fromDegrees(0)),
                    new Pose2d(startPose.getX() + 0.50, startPose.getY(), Rotation2d.fromDegrees(0)),
                    new Pose2d(startPose.getX() + 0.25, startPose.getY() + 0.25, Rotation2d.fromDegrees(0))
                )
            );

            Action driveTestPathAction = new DrivePathAction(trajectoryPoints);

            return driveTestPathAction.withSubsystem(Auto.getInstance());
        }

        // public static final Action blueBottomCubeAutonomous() {
        //     Action blueBottomCubeRetrieve = new DrivePathAction("Blue Bottom Cube Retrieve");

        //     Action getBlueBottomCube = new Action(
        //         () -> {}, 
        //         () -> {
        //             System.out.println("Ran");
                    
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

        //                 Gripper.getInstance().enableGripper();
        //             }
        //         }, 
        //         () -> {}, 
        //         ActionConstants.WILL_CANCEL
        //     );

        //     Action blueBottomCubeScore = new DrivePathAction("Blue Bottom Cube Score");

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

        //             Gripper.getInstance().disableGripper();
        //         }, 
        //         () -> {
        //             Extension.getInstance().setLowerExtensionState(LowerExtensionState.OFF);
        //         }, 
        //         ActionConstants.WILL_CANCEL
        //     );

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