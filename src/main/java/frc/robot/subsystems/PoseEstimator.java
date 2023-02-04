package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;

import frc.libs.java.actionLib.*;
import frc.robot.Robot;
import frc.robot.Constants.*;

public class PoseEstimator extends Subsystem {
    public static PoseEstimator mInstance;

    Pose2d initialPose = new Pose2d();

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
        return this.swervePoseEstimator.getEstimatedPosition();
    }

    public Rotation2d getSwerveRotation() {
        return this.swervePoseEstimator.getEstimatedPosition().getRotation();
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
        SmartDashboard.putNumber("Robot Pose - Angle", this.getSwerveRotation().getDegrees());
    }

    @Override
    public void periodic() {
        PoseEstimator.getInstance().swervePoseEstimator.update(Swerve.getInstance().getGyroRotation(), Swerve.getInstance().getModulePositions());
        PoseEstimator.getInstance().swervePoseEstimator.addVisionMeasurement(PoseEstimator.getInstance().getVisionPose2d(), Timer.getFPGATimestamp());
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
