package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.libs.java.actions.*;
import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.helpers.LimelightHelper;

public final class PoseEstimator extends Subsystem {
    private static PoseEstimator mInstance;
    private Field2d mField2d = new Field2d();
    Pose2d initialPose = new Pose2d();

    public SwerveDrivePoseEstimator swervePoseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.SWERVE_KINEMATICS, 
        Swerve.getInstance().getGyroRotation(),
        Swerve.getInstance().getModulePositions(),
        initialPose
    );

    private PoseEstimator() {
    }

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

            Pose2d pose;

            if (alliance == Alliance.Red) {
                pose = LimelightHelper.getBotPose2d_wpiRed(limelightName);
            } else {
                pose = LimelightHelper.getBotPose2d_wpiBlue(limelightName);
            }

            double trust = (1 - LimelightHelper.getTA(limelightName)) * 15;
            double latency = LimelightHelper.getLatency_Pipeline(limelightName) / 1000.0;

            swervePoseEstimator.setVisionMeasurementStdDevs(new MatBuilder<>(Nat.N3(), Nat.N1()).fill(trust, trust, trust));
            swervePoseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - latency);
        }
    }

    public void updatePoseEstimator() {
        swervePoseEstimator.update(Rotation2d.fromDegrees(Swerve.getInstance().getGyroRotation().getDegrees()), Swerve.getInstance().getModulePositions());
        addVisionPose2d();

        mField2d.setRobotPose(getSwervePose());

        SmartDashboard.putNumber("Robot Pose - X", getSwervePose().getX());
        SmartDashboard.putNumber("Robot Pose - Y", getSwervePose().getY());
        SmartDashboard.putNumber("Robot Pose - Angle", getSwerveRotation().getDegrees());
        SmartDashboard.putData("Field", mField2d);
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
