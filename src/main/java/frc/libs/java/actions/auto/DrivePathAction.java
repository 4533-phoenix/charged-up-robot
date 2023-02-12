package frc.libs.java.actions.auto;

import frc.libs.java.actions.*;
import frc.libs.java.swerve.SwervePath;
import frc.libs.java.swerve.SwervePath.PathState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;

public final class DrivePathAction extends Action {
    private final SwervePath mPath;

    public DrivePathAction(String path) {
        super(() -> {}, () -> {}, () -> {}, ActionConstants.WILL_CANCEL);

        this.mPath = SwervePath.fromCSV(path);
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

        while (Timer.getFPGATimestamp() - startTime <= this.mPath.getRuntime()) {
            PathState currState = this.mPath.sample(Timer.getFPGATimestamp() - startTime);

            ChassisSpeeds chassisSpeeds = Auto.getInstance().getAutoController().calculate(
                PoseEstimator.getInstance().getSwervePose(),
                currState.getTrajectoryState(),
                currState.rotation
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
