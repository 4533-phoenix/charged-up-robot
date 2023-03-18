package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.Arrays;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.libs.java.actions.*;
import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.helpers.LimelightHelper;

public final class PoseEstimator extends Subsystem {
    private static PoseEstimator mInstance;

    Pose2d initialPose = new Pose2d();

    public SwerveDrivePoseEstimator swervePoseEstimator = new SwerveDrivePoseEstimator(
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

    public void addVisionPose2d() {
        Alliance alliance = DriverStation.getAlliance();

        for (String limelightName : LimelightConstants.LIMELIGHT_NAMES) {
            if (!LimelightHelper.getTV(limelightName)) {
                continue;
            }

            Pose2d pose = new Pose2d();

            if (alliance == Alliance.Red) {
                pose = LimelightHelper.getBotPose2d_wpiRed(limelightName);
            } else if (alliance == Alliance.Blue) {
                pose = LimelightHelper.getBotPose2d_wpiBlue(limelightName);
            } else {
                pose = LimelightHelper.getBotPose2d(limelightName);
            }

            PoseEstimator.getInstance().swervePoseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - LimelightHelper.getLatency_Pipeline(limelightName));
        }
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
        addVisionPose2d();
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
