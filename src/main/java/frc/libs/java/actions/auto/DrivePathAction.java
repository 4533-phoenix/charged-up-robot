package frc.libs.java.actions.auto;

import frc.libs.java.actions.*;
import edu.wpi.first.math.trajectory.*;
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

    private Rotation2d rotation;

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

        boolean autoDone = false;

        System.out.println("total time: " + this.mTrajectory.getTotalTimeSeconds());

        while (Timer.getFPGATimestamp() - startTime <= this.mTrajectory.getTotalTimeSeconds()) {
            Trajectory.State currState = this.mTrajectory.sample(Timer.getFPGATimestamp() - startTime);

            rotation = Rotation2d.fromRadians(currState.curvatureRadPerMeter * currState.velocityMetersPerSecond);

            if (!autoDone) {
                ChassisSpeeds chassisSpeeds = Auto.getInstance().getAutoController().calculate(
                    PoseEstimator.getInstance().getSwervePose(),
                    currState,
                    rotation
                );

                SwerveModuleState[] swerveModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

                Swerve.getInstance().setModuleStates(swerveModuleStates);
            }
        }

        autoDone = true;

        System.out.println("done. time: " + (Timer.getFPGATimestamp() - startTime));
        
        Swerve.getInstance().setModuleStates(DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds()));

        if (this.willThreadRun()) {
            try {
                this.getThreadLock().unlock();
            }
            finally {}
        }
    }
}
