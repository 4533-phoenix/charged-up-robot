package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.Robot;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.libs.java.actions.Action;
import frc.libs.java.actions.Subsystem;
import frc.libs.java.actions.auto.*;
import frc.robot.subsystems.Extension.ExtensionState;
import java.util.Map;

public final class Auto extends Subsystem {
    private static Auto mInstance;

    public Pose2d startPose;

    private static final Map<String, Action> autoCommands = Map.ofEntries(
        Map.entry("PathPlanner Test", AutoActions.pathPlannerTest()),
        Map.entry("Right/Left Score and Leave", AutoActions.rightLeftScoreAndLeave()),
        Map.entry("Charge Station Score and Enable", AutoActions.chargeStationScoreAndEnable()),
        Map.entry("Do Nothing", AutoActions.doNothing())
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

    public void enableChargeStation(boolean direction) {
        ChassisSpeeds driveSpeeds;
       
        if (direction) {
            driveSpeeds =  new ChassisSpeeds(Units.feetToMeters(6.5), 0, 0);
        } else {
            driveSpeeds = new ChassisSpeeds(Units.feetToMeters(-6.5), 0, 0);
        }

        SwerveModuleState[] moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(driveSpeeds);

        Swerve.getInstance().setModuleStates(moduleStates);

        while (Math.abs(Swerve.getInstance().getPitch()) < 8.0) {
            moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(driveSpeeds);

            Swerve.getInstance().setModuleStates(moduleStates);
        }

        if (direction) {
            driveSpeeds =  new ChassisSpeeds(Units.feetToMeters(2.0), 0, 0);
        } else {
            driveSpeeds = new ChassisSpeeds(Units.feetToMeters(-2.0), 0, 0);
        }

        while (Math.abs(Swerve.getInstance().getPitch()) > 2.0) {
            moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(driveSpeeds);

            Swerve.getInstance().setModuleStates(moduleStates);
        }
        
        Swerve.getInstance().setModuleStates(DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds()));
    }

    public void adjustChargeStation() {
        while (Math.abs(Swerve.getInstance().getPitch()) > DriveConstants.CHARGE_STATION_PITCH_DEADBAND) {
            ChassisSpeeds driveSpeeds = Swerve.getInstance().getPitch() < 0.0 ? new ChassisSpeeds(-0.5, 0.0, 0.0) : new ChassisSpeeds(0.5, 0.0, 0.0);

            Swerve.getInstance().setModuleStates(DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(driveSpeeds));
        }

        Swerve.getInstance().setModuleStates(DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds()));
    }

    private static final class AutoActions {
        public static final Action doNothing() {
            return new SeriesAction();
        }

        public static final Action pathPlannerTest() {
            DrivePathAction testPath = new DrivePathAction("PathPlanner Test", 5.0, 3.0, false);
            Auto.getInstance().startPose = testPath.getStartPose();

            return testPath.withSubsystem(Auto.getInstance());
        }

        public static final Action rightLeftScoreAndLeave() {
            Action testAuto = new SeriesAction(
                new LambdaAction(() -> Extension.getInstance().updateExtensionState(ExtensionState.HIGH_ROW)),
                new LambdaAction(() -> Gripper.getInstance().enableGripper()),
                new LambdaAction(() -> Swerve.getInstance().setModuleStates(DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds()))),
                new WaitAction(3.0),
                new LambdaAction(() -> Gripper.getInstance().disableGripper()),
                new WaitAction(0.5),
                new DriveDistanceAction(-1.0),
                new LambdaAction(() -> Extension.getInstance().updateExtensionState(ExtensionState.OFF_GROUND)),
                new DriveDistanceAction(-4.0)
            );

            return testAuto.withSubsystem(Auto.getInstance());
        }

        public static final Action chargeStationScoreAndEnable() {
            Action testAuto = new SeriesAction(
                new LambdaAction(() -> Gripper.getInstance().enableGripper()),
                new LambdaAction(() -> Extension.getInstance().updateExtensionState(ExtensionState.MIDDLE_ROW)),
                new LambdaAction(() -> Swerve.getInstance().setModuleStates(DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds()))),
                new WaitAction(4.0),
                new LambdaAction(() -> Gripper.getInstance().disableGripper()),
                new WaitAction(0.5),
                new LambdaAction(() -> Auto.getInstance().enableChargeStation(false)),
                new WaitAction(0.5),
                new LambdaAction(() -> Auto.getInstance().adjustChargeStation())
            );

            return testAuto.withSubsystem(Auto.getInstance());
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