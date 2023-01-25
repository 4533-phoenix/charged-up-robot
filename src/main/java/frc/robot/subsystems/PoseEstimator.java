package frc.robot.subsystems;

import frc.robot.loops.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.Constants.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;

public class PoseEstimator extends Subsystem {
    public static PoseEstimator mInstance;

    Swerve mSwerve = Swerve.getInstance();

    Pose2d initialPose = new Pose2d();

    Pose2d currentPose = new Pose2d();

    private SwerveDrivePoseEstimator swervePoseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.SWERVE_KINEMATICS, 
        mSwerve.getGyroRotation(),
        mSwerve.getModulePositions(),
        initialPose
    );

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

    public PoseEstimator() {}

    private static final class PoseEstimatorLoops {
        private PoseEstimator mPoseEstimator = PoseEstimator.getInstance();
        private Swerve mSwerve = Swerve.getInstance();

        public Loop poseEstimationLoop() {
            return new Loop() {
                @Override
                public void onStart(double timestamp) {}

                @Override
                public void onLoop(double timestamp) {
                    mPoseEstimator.currentPose = 
                        mPoseEstimator.swervePoseEstimator.update(
                            mSwerve.getGyroRotation(),
                            mSwerve.getModulePositions()
                        );
                }

                @Override
                public void onStop(double timestamp) {}
            };
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        PoseEstimatorLoops poseEstimatorLoops = new PoseEstimatorLoops();

        mEnabledLooper.register(poseEstimatorLoops.poseEstimationLoop());
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void writeToDashboard() {
        SmartDashboard.putNumber("Robot Pose - X", this.currentPose.getX());
        SmartDashboard.putNumber("Robot Pose - Y", this.currentPose.getY());
        SmartDashboard.putNumber("Robot Pose - Angle", this.currentPose.getRotation().getDegrees());
    }
}
