package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.*;

public class DriveDistanceCommand extends Command {
    Swerve mSwerve;

    private double startTime;
    private double totalTime;
    private double distanceMeters;

    private boolean isFinished = false;

    public DriveDistanceCommand(double distanceMeters, Swerve swerve) {
        this.mSwerve = swerve;

        this.distanceMeters = distanceMeters;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        this.startTime = Timer.getFPGATimestamp();

        this.totalTime = Math.abs(this.distanceMeters) / AutoConstants.AUTO_MAX_VELOCITY;
    }

    @Override
    public void execute() {
        this.startTime = Timer.getFPGATimestamp();

        ChassisSpeeds driveSpeeds;
       
        if (this.distanceMeters >= 0) {
            driveSpeeds =  new ChassisSpeeds(-AutoConstants.AUTO_MAX_VELOCITY, 0, 0);
        } else {
            driveSpeeds = new ChassisSpeeds(AutoConstants.AUTO_MAX_VELOCITY, 0, 0);
        }

        SwerveModuleState[] moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(driveSpeeds);

        mSwerve.setModuleStates(moduleStates);

        if (Timer.getFPGATimestamp() - startTime >= this.totalTime) {
            this.isFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        mSwerve.stopDrive();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
