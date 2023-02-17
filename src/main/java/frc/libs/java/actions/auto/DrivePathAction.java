package frc.libs.java.actions.auto;

import frc.libs.java.actions.*;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import java.util.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class DrivePathAction extends Action {
    private final Trajectory mTrajectory;

    private TrajectoryConfig config = new TrajectoryConfig(AutoConstants.AUTO_MAX_VELOCITY, AutoConstants.AUTO_MAX_ACCELERATION);

    public DrivePathAction(List<Pose2d> waypoints) {
        super(() -> {}, () -> {}, () -> {}, ActionConstants.WILL_CANCEL);

        this.mTrajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);
    }

    @Override
    public void run() {
        if (this.willThreadRun()) {
            try {
                this.getThreadLock().lock();
            }
            finally {}
        }

        double startTime = Timer.getFPGATimestamp();

        while (Timer.getFPGATimestamp() - startTime <= this.mTrajectory.getTotalTimeSeconds()) {
            Trajectory.State currState = this.mTrajectory.sample(Timer.getFPGATimestamp() - startTime);

            ChassisSpeeds chassisSpeeds = Auto.getInstance().getAutoController().calculate(
                PoseEstimator.getInstance().getSwervePose(),
                currState,
                Rotation2d.fromDegrees(0.0)
            );

            SwerveModuleState[] swerveModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

            Swerve.getInstance().setModuleStates(swerveModuleStates);
        }
        
        Swerve.getInstance().setModuleStates(DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds()));

        if (this.willThreadRun()) {
            try {
                this.getThreadLock().unlock();
            }
            finally {}
        }
    }
}
