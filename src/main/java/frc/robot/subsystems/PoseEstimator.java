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

    Pose2d initialPose = this.getVisionPose2d();
    
    NetworkTable table = NetworkTableInstance.getDefault().getTable("apriltag");
    NetworkTableEntry position = table.getEntry("position");
    NetworkTableEntry rotation = table.getEntry("rotation");

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
        return new Pose2d(this.swervePoseEstimator.getEstimatedPosition().getX(),
            this.swervePoseEstimator.getEstimatedPosition().getY(), this.swervePoseEstimator.getEstimatedPosition().getRotation());
    }

    public Pose2d getOdometrySwervePose() {
        return this.swervePoseEstimator.getOdometry().getPoseMeters();
    }

    public void resetSwervePose() {
        this.swervePoseEstimator.resetPosition(getSwerveRotation(), Swerve.getInstance().getModulePositions(), new Pose2d());
    }

    public Rotation2d getSwerveRotation() {
        return this.getSwervePose().getRotation();
    }

    public void resetPoseEstimator(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        this.swervePoseEstimator.resetPosition(gyroAngle, modulePositions, this.initialPose);
    }

    public Translation3d getPositionFromVision() {
        double[] defaultArray = {0,0,0};
        double[] positionArray = position.getDoubleArray(defaultArray);

        return new Translation3d(positionArray[0], positionArray[1], positionArray[2]);
    }

    public Rotation2d getRotationFromVision() {
        double rotationFromTable = rotation.getDouble(0);

        return new Rotation2d(rotationFromTable);
    }

    public Pose2d getVisionPose2d() {
        return new Pose2d(this.getPositionFromVision().getX(), this.getPositionFromVision().getY(), getRotationFromVision());
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
        // PoseEstimator.getInstance().swervePoseEstimator.addVisionMeasurement(PoseEstimator.getInstance().getVisionPose2d(), Timer.getFPGATimestamp());
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
