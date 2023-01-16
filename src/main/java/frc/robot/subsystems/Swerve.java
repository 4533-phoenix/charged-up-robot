package frc.robot.subsystems;

import frc.robot.controls.*;
import frc.robot.controls.PSController.Button;
import frc.robot.loops.*;
import frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.MatBuilder;

public final class Swerve extends Subsystem {
    private static Swerve mInstance;

    private SwerveModule[] swerveMods;

    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
        DriveConstants.FRONT_LEFT_STEER_MOTOR_ID,
        DriveConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED,
        DriveConstants.FRONT_LEFT_STEER_ENCODER_REVERSED,
        DriveConstants.FRONT_LEFT_STEER_ABSOLUTE_ENCODER_ID,
        DriveConstants.FRONT_LEFT_STEER_ABSOLUTE_ENCODER_OFFSET,
        DriveConstants.FRONT_LEFT_STEER_ABSOLUTE_ENCODER_REVERSED
    );

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
        DriveConstants.FRONT_RIGHT_STEER_MOTOR_ID,
        DriveConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED,
        DriveConstants.FRONT_RIGHT_STEER_ENCODER_REVERSED,
        DriveConstants.FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_ID,
        DriveConstants.FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_OFFSET,
        DriveConstants.FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_REVERSED
    );

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.BACK_LEFT_DRIVE_MOTOR_ID,
        DriveConstants.BACK_LEFT_STEER_MOTOR_ID,
        DriveConstants.BACK_LEFT_DRIVE_ENCODER_REVERSED,
        DriveConstants.BACK_LEFT_STEER_ENCODER_REVERSED,
        DriveConstants.BACK_LEFT_STEER_ABSOLUTE_ENCODER_ID,
        DriveConstants.BACK_LEFT_STEER_ABSOLUTE_ENCODER_OFFSET,
        DriveConstants.BACK_LEFT_STEER_ABSOLUTE_ENCODER_REVERSED
    );

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.BACK_RIGHT_DRIVE_MOTOR_ID,
        DriveConstants.BACK_RIGHT_STEER_MOTOR_ID,
        DriveConstants.BACK_RIGHT_DRIVE_ENCODER_REVERSED,
        DriveConstants.BACK_RIGHT_STEER_ENCODER_REVERSED,
        DriveConstants.BACK_RIGHT_STEER_ABSOLUTE_ENCODER_ID,
        DriveConstants.BACK_RIGHT_STEER_ABSOLUTE_ENCODER_OFFSET,
        DriveConstants.BACK_RIGHT_STEER_ABSOLUTE_ENCODER_REVERSED
    );

    private SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.DRIVE_MAX_ACCELERATION);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.DRIVE_MAX_ACCELERATION);
    private SlewRateLimiter steerLimiter = new SlewRateLimiter(DriveConstants.DRIVE_MAX_ROTATIONAL_ACCELERATION);

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    private Pose2d swervePose = new Pose2d();

    private SwerveDrivePoseEstimator swervePoseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.SWERVE_KINEMATICS, 
        Rotation2d.fromDegrees(-this.gyro.getAngle()),
        this.getModulePositions(),
        this.swervePose
    );
    
    public static Swerve getInstance() {
        if (mInstance == null) {
            mInstance = new Swerve();
        }
        return mInstance;
    }

    public Swerve() {
        this.zeroGyro();

        this.swerveMods = new SwerveModule[] {
           frontLeft,
           frontRight,
           backLeft,
           backRight 
        };
    }

    public void zeroGyro() {
        this.gyro.reset();
    }

    public Pose2d getSwervePose() {
        return this.swervePose;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            this.frontLeft.getModulePosition(),
            this.frontRight.getModulePosition(),
            this.backLeft.getModulePosition(),
            this.backRight.getModulePosition()
        };
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.DRIVE_MAX_VELOCITY);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        double xSpeed = translation.getX();
        double ySpeed = translation.getY();
        double steerSpeed = rotation;

        xSpeed = Math.abs(xSpeed) > OIConstants.DRIVE_DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.DRIVE_DEADBAND ? ySpeed : 0.0;
        steerSpeed = Math.abs(steerSpeed) > OIConstants.DRIVE_DEADBAND ? steerSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        steerSpeed = steerLimiter.calculate(steerSpeed);

        ChassisSpeeds chassisSpeeds;
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, steerSpeed, Rotation2d.fromDegrees(-this.gyro.getAngle())
            );
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, steerSpeed);
        }

        SwerveModuleState[] moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        this.setModuleStates(moduleStates);
    }

    private static final class SwerveLoops {
        public Loop defaultDriveLoop() {
            return new Loop() {
                @Override
                public void onStart(double timestamp) {
                    Swerve.getInstance().drive(new Translation2d(), 0.0, false, true);
                }

                @Override
                public void onLoop(double timestamp) {
                    if (!Auto.getInstance().isEnabled()) {
                        Translation2d swerveTranslation = DriveController.getInstance().getSwerveTranslation();
                        double swerveRotation = DriveController.getInstance().getSwerveRotation();

                        Swerve.getInstance().drive(swerveTranslation, swerveRotation, false, true);
                    }
                }

                @Override
                public void onStop(double timestamp) {
                    Swerve.getInstance().drive(new Translation2d(), 0.0, false, true);
                }
            };
        }

        public Loop swervePeriodic() {
            return new Loop() {
                @Override
                public void onStart(double timestamp) {}

                @Override
                public void onLoop(double timestamp) {
                    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

                    for (int i = 0; i < 4; i++) {
                        swerveModuleStates[i] = Swerve.getInstance().swerveMods[i].getState();
                    }

                    Swerve.getInstance().swervePose = 
                        Swerve.getInstance().swervePoseEstimator.update(
                            Rotation2d.fromDegrees(-Swerve.getInstance().gyro.getAngle()),
                            Swerve.getInstance().getModulePositions()
                        );

                    Swerve.getInstance().writeToDashboard();
                }

                @Override
                public void onStop(double timestamp) {}
            };
        }

        public Loop startButton() {
            return new Loop() {
                @Override
                public void onStart(double timestamp) {}

                @Override
                public void onLoop(double timestamp) {
                    if (DriveController.getInstance().getButton(Button.START)) {
                        Swerve.getInstance().zeroGyro();
                    }
                }

                @Override
                public void onStop(double timestamp) {}
            };
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        SwerveLoops swerveLoops = new SwerveLoops();

        mEnabledLooper.register(swerveLoops.defaultDriveLoop());
        mEnabledLooper.register(swerveLoops.swervePeriodic());
        mEnabledLooper.register(swerveLoops.startButton());
    }

    @Override
    public void stop() {
        // TODO: Add code that will fully stop this subsystem
    }

    @Override
    public boolean checkSystem() {
        // TODO: Add code that checks possible system faults (should be SERIOUS FAULTS)
        // Serious faults should be things that could damage or hurt other people/things
        // Serious faults due to the fact that stop() will most likely be called
        // after this returning false
        return true;
    }

    @Override
    public void writeToDashboard() {
        SmartDashboard.putNumber("Robot Pose - X", this.swervePose.getX());
        SmartDashboard.putNumber("Robot Pose - Y", this.swervePose.getY());
        SmartDashboard.putNumber("Robot Pose - Angle", this.swervePose.getRotation().getDegrees());

        SmartDashboard.putNumber("Gyro Heading", this.swervePose.getRotation().getDegrees());
        SmartDashboard.putNumber("Gyro Pitch", this.gyro.getPitch());
        SmartDashboard.putNumber("Gyro Roll", this.gyro.getRoll());
    }
}
