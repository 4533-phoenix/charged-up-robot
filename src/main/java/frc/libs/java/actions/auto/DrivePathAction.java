package frc.libs.java.actions.auto;

import frc.libs.java.actions.*;
import com.pathplanner.lib.*;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class DrivePathAction extends Action {
    private PathPlannerTrajectory mPath;
    private Pose2d startPose;

    public DrivePathAction(String path, double maxVelocity, double maxAccel, boolean isReversed) {
        super(() -> {}, () -> {}, () -> {}, true);

        Auto.getInstance().getAutoController().setEnabled(true);

        this.mPath = PathPlanner.loadPath(path, maxVelocity, maxAccel, isReversed);
        this.startPose = mPath.getInitialHolonomicPose();
    }

    @Override
    public void run() {
        double startTime = Timer.getFPGATimestamp();

        this.mPath = PathPlannerTrajectory.transformTrajectoryForAlliance(this.mPath, DriverStation.getAlliance());

        while (Timer.getFPGATimestamp() <= (this.mPath.getEndState().timeSeconds * 1.15)  + startTime) {
            PathPlannerState currState = (PathPlannerState) this.mPath.sample(Timer.getFPGATimestamp() - startTime);
            currState = PathPlannerTrajectory.transformStateForAlliance(currState, DriverStation.getAlliance());

            ChassisSpeeds chassisSpeeds = Auto.getInstance().getAutoController().calculate(
                PoseEstimator.getInstance().getSwervePose(),
                currState
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