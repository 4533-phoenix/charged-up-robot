package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;
import frc.libs.java.actions.*;
import frc.robot.Robot;
import frc.robot.Constants.*;

public final class PoseEstimator extends Subsystem {
    private static PoseEstimator mInstance;

    Pose2d defaultPose = new Pose2d();
    
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry pipeline = table.getEntry("getPipe");

    NetworkTableEntry apriltagBotPose = table.getEntry("botpose");
    NetworkTableEntry targetDetected = table.getEntry("tv");

    if (pipeline.getDouble() == 0) {
        // apriltag pipeline
    }

    private SwerveDrivePoseEstimator swervePoseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.SWERVE_KINEMATICS, 
        Swerve.getInstance().getGyroRotation(),
        Swerve.getInstance().getModulePositions(),
        this.defaultPose
    );

    private PoseEstimator() {}

    public static PoseEstimator getInstance() {
        if (mInstance == null) {
            mInstance = new PoseEstimator();
        }

        return mInstance;
    }

    public Pose2d getSwervePose() {
        return new Pose2d(this.swervePoseEstimator.getEstimatedPosition().getX(),
            this.swervePoseEstimator.getEstimatedPosition().getY(), this.swervePoseEstimator.getEstimatedPosition().getRotation());
    }

    public Pose2d getOdometrySwervePose() {
        return this.swervePoseEstimator.getOdometry().getPoseMeters();
    }

    public void resetSwervePose() {
        this.swervePoseEstimator.resetPosition(getSwerveRotation(), Swerve.getInstance().getModulePositions(), this.defaultPose);
    }

    public void addApriltagPose() {
        // if the apriltag is detected, add the apriltag pose to the swerve pose using addVisionMeasurement
        // botpose: Robot transform in field-space. Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
        if (targetDetected.getDouble() == 1) {
            double[] botpose = apriltagBotPose.getDoubleArray(new double[7]);
            this.swervePoseEstimator.addVisionMeasurement(new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5])), Timer.getFPGATimestamp() - botpose[6]);
        }
    }

    public Rotation2d getSwerveRotation() {
        return this.getSwervePose().getRotation();
    }

    public void resetPoseEstimator(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        this.swervePoseEstimator.resetPosition(gyroAngle, modulePositions, this.defaultPose);
    }

    @Override
    public void log() {
        SmartDashboard.putNumber("Robot Pose - X", this.getSwervePose().getX());
        SmartDashboard.putNumber("Robot Pose - Y", this.getSwervePose().getY());
        SmartDashboard.putNumber("Odometry Robot Pose - X" , this.getOdometrySwervePose().getX());
        SmartDashboard.putNumber("Odometry Robot Pose - Y" , this.getOdometrySwervePose().getY());
        SmartDashboard.putNumber("Robot Pose - Angle", this.getSwerveRotation().getDegrees());
    }

    @Override
    public void periodic() {
        PoseEstimator.getInstance().swervePoseEstimator.update(Rotation2d.fromDegrees(Swerve.getInstance().getGyroRotation().getDegrees() + Swerve.getInstance().initialGyroOffset), Swerve.getInstance().getModulePositions());
        PoseEstimator.getInstance().addApriltagPose();
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
