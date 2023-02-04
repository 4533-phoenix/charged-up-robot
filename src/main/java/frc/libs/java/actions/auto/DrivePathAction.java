package frc.libs.java.actions.auto;

import frc.libs.java.actions.*;
import frc.libs.java.swerve.SwervePath;
import frc.libs.java.swerve.SwervePath.PathState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;

public class DrivePathAction extends Action {
    private final SwervePath mPath;
    private double mStartTime;

    public DrivePathAction(String path) {
        super(Auto.getEmptyRunnable(), Auto.getEmptyRunnable(), Auto.getEmptyRunnable(), true);

        this.mPath = SwervePath.fromCSV(path);
    }

    @Override
    public void runStart() {
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void run() {
        PathState currState = mPath.sample(Timer.getFPGATimestamp() - mStartTime);

        ChassisSpeeds chassisSpeeds = Auto.getInstance().getAutoController().calculate(
            PoseEstimator.getInstance().getSwervePose(),
            currState.getTrajectoryState(),
            currState.rotation
        );

        SwerveModuleState[] swerveModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        Swerve.getInstance().setModuleStates(swerveModuleStates);
    }

    @Override
    public void runEnd() {}

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime >= mPath.getRuntime();
    }
}
