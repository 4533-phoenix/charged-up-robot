package frc.libs.java.actions.auto;

import frc.libs.java.actions.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;

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

        this.totalTime = Math.abs(this.distanceMeters) / AutoConstants.AUTO_MAX_VELOCITY;

        System.out.println("driving distance");

        ChassisSpeeds driveSpeeds;
       
        if (this.distanceMeters >= 0) {
            driveSpeeds =  new ChassisSpeeds(AutoConstants.AUTO_MAX_VELOCITY, 0, 0);
        } else {
            driveSpeeds = new ChassisSpeeds(-AutoConstants.AUTO_MAX_VELOCITY, 0, 0);
        }

        SwerveModuleState[] moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(driveSpeeds);

        Swerve.getInstance().setModuleStates(moduleStates);

        while (Timer.getFPGATimestamp() - startTime <= this.totalTime) {
            moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(driveSpeeds);

            Swerve.getInstance().setModuleStates(moduleStates);
        }
        
        Swerve.getInstance().setModuleStates(DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds()));

        this.isFinished = true;

        if (this.willThreadRun()) {
            try {
                this.getThreadLock().unlock();
            }
            finally {}
        }
    }
}
