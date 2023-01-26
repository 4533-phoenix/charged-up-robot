package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.libs.java.actionLib.*;
import frc.robot.Robot;
import frc.robot.Constants.*;

public class PoseEstimator extends Subsystem {
    public static PoseEstimator mInstance;

    Pose2d initialPose = new Pose2d();

    Pose2d currentPose = new Pose2d();

    private SwerveDrivePoseEstimator swervePoseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.SWERVE_KINEMATICS, 
        Swerve.getInstance().getGyroRotation(),
        Swerve.getInstance().getModulePositions(),
        initialPose
    );

    private PoseEstimator() {}

    public static PoseEstimator getInstance() {
        if (mInstance == null) {
            mInstance = new PoseEstimator();
        }
        return mInstance;
    }

    public Pose2d getSwervePose() {
        return this.currentPose;
    }

    public Rotation2d getSwerveRotation() {
        return this.currentPose.getRotation();
    }

    private static final class PoseEstimatorActions {}

    @Override
    public void log() {
        SmartDashboard.putNumber("Robot Pose - X", this.currentPose.getX());
        SmartDashboard.putNumber("Robot Pose - Y", this.currentPose.getY());
        SmartDashboard.putNumber("Robot Pose - Angle", this.currentPose.getRotation().getDegrees());
    }

    @Override
    public void periodic() {
        this.currentPose = this.swervePoseEstimator.update(
            Swerve.getInstance().getGyroRotation(),
            Swerve.getInstance().getModulePositions()
        );
    }

    @Override
    public void queryInitialActions() {
        Robot.autonomousRunner.add(
            this.getLoggingAction(),
            this.getPeriodicAction()
        );

        Robot.teleopRunner.add(
            this.getLoggingAction(),
            this.getPeriodicAction()
        );
    }
}
