package frc.libs.java.actions.auto;

import frc.libs.java.actions.*;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;

public class DrivePathAction extends Action {
    private final PathPlannerTrajectory mPath;
    private Pose2d startPose;

    public DrivePathAction(String path, double maxVelocity, double maxAccel, boolean isReversed) {
        super(() -> {}, () -> {}, () -> {}, true);

        this.mPath = PathPlanner.loadPath(path, maxVelocity, maxAccel, isReversed);
        this.startPose = mPath.getInitialPose();
    }

    @Override
    public void run() {
        double startTime = Timer.getFPGATimestamp();

        while (Timer.getFPGATimestamp() <= this.mPath.getEndState().timeSeconds + startTime) {
            System.out.println("running path");

            Trajectory.State currState = this.mPath.sample(Timer.getFPGATimestamp() - startTime);

            ChassisSpeeds chassisSpeeds = Auto.getInstance().getAutoController().calculate(
                PoseEstimator.getInstance().getSwervePose(),
                currState,
                new Rotation2d()
            );

            SwerveModuleState[] swerveModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

            Swerve.getInstance().setModuleStates(swerveModuleStates);
        }
        
        Swerve.getInstance().setModuleStates(DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds()));

        this.isFinished = true;
    }

    public Pose2d getStartPose() {
        return this.startPose;
    }
}