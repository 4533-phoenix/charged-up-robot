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

public final class DriveDistanceAction extends Action {
    private double startTime;
    private double totalTime;
    private double distanceMeters;

    public DriveDistanceAction(double distanceMeters) {
        super(() -> {}, () -> {}, () -> {}, ActionConstants.WILL_CANCEL);

        this.distanceMeters = distanceMeters;
    }

    @Override
    public void run() {
        if (this.willThreadRun()) {
            try {
                this.getThreadLock().lock();
            }
            finally {}
        }

        this.startTime = Timer.getFPGATimestamp();

        this.totalTime = this.distanceMeters / AutoConstants.AUTO_MAX_VELOCITY;

        SwerveModuleState everyState;

        if (this.distanceMeters >= 0) {
            everyState = new SwerveModuleState(AutoConstants.AUTO_MAX_VELOCITY, new Rotation2d());
        } else {
            everyState = new SwerveModuleState(AutoConstants.AUTO_MAX_VELOCITY, Rotation2d.fromDegrees(180));
        }

        Swerve.getInstance().setAllModuleStates(everyState);

        while (Timer.getFPGATimestamp() - startTime <= this.totalTime) {}
        
        Swerve.getInstance().setModuleStates(DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds()));

        if (this.willThreadRun()) {
            try {
                this.getThreadLock().unlock();
            }
            finally {}
        }
    }
}
