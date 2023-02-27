package frc.robot.subsystems;

import frc.robot.controls.PSController.Axis;
import frc.robot.controls.PSController.Button;
import frc.robot.controls.PSController.Side;
import frc.libs.java.actions.Action;
import frc.libs.java.actions.Subsystem;
import frc.robot.Robot;
import frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    public double gyroOffset;
    
    private Swerve() {
        this.zeroGyro();

        this.swerveMods = new SwerveModule[] {
           frontLeft,
           frontRight,
           backLeft,
           backRight 
        };
    }

    public static Swerve getInstance() {
        if (mInstance == null) {
            mInstance = new Swerve();
        }

        return mInstance;
    }

    public void zeroGyro() {
        this.gyro.reset();
    }

    public void zeroYaw() {
        this.gyro.zeroYaw();
    }

    public Rotation2d getGyroRotation() {
        // double angle = -this.gyro.getYaw();

        // angle *= Math.PI / 180.0;
        // angle %= 2.0 * Math.PI;
        // angle += gyroOffset;

        // if (angle < 0.0)
        //     angle += 2.0 * Math.PI;

        return Rotation2d.fromDegrees(-this.gyro.getYaw());
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
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.DRIVE_MAX_PHYSICAL_VELOCITY);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        double xSpeed = DriveConstants.DRIVE_MAX_VELOCITY * translation.getX();
        double ySpeed = DriveConstants.DRIVE_MAX_VELOCITY * translation.getY();
        double steerSpeed = DriveConstants.DRIVE_MAX_ROTATIONAL_VELOCITY * rotation;

        xSpeed = Math.abs(xSpeed) > OIConstants.DRIVE_DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.DRIVE_DEADBAND ? ySpeed : 0.0;
        steerSpeed = Math.abs(steerSpeed) > OIConstants.DRIVE_DEADBAND ? steerSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        steerSpeed = steerLimiter.calculate(steerSpeed);

        ChassisSpeeds chassisSpeeds;
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, steerSpeed, getGyroRotation()
            );
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, steerSpeed);
        }

        SwerveModuleState[] moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        this.setModuleStates(moduleStates);
    }

    public Translation2d getSwerveTranslation() {
        double forwardAxis = -Robot.driverController.getAxis(Side.LEFT, Axis.Y);
        double strafeAxis = -Robot.driverController.getAxis(Side.LEFT, Axis.X);

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        double dist = tAxes.getNorm();

        if (Math.abs(tAxes.getNorm()) < OIConstants.DRIVE_DEADBAND) {
            return new Translation2d();
        } else {
            return new Translation2d(forwardAxis * dist, strafeAxis * dist);
        }
    }

    public double getSwerveRotation() {
        double rotAxis = -Math.pow(Robot.driverController.getAxis(Side.RIGHT, Axis.X), 3);

        if (Math.abs(rotAxis) < OIConstants.DRIVE_DEADBAND) {
            return 0.0;
        } else {
            return rotAxis;
        }
    }

    private static final class SwerveActions {
        public static final Action defaultDriveAction() {
            Runnable startMethod = () -> {
                Swerve.getInstance().drive(new Translation2d(), 0.0, true, true);
                Swerve.getInstance().gyroOffset = Swerve.getInstance().getGyroRotation().getDegrees();
            };

            Runnable runMethod = () -> {
                Translation2d swerveTranslation = Swerve.getInstance().getSwerveTranslation();
                
                double swerveRotation = Swerve.getInstance().getSwerveRotation();

                Swerve.getInstance().drive(swerveTranslation, swerveRotation, true, true);
            };

            Runnable endMethod = () -> {
                Swerve.getInstance().drive(new Translation2d(), 0.0, true, true);
            };

            return new Action(startMethod, runMethod, endMethod, ActionConstants.WILL_NOT_CANCEL);
        }

        public static final Action startButtonAction() {
            Runnable startMethod = () -> {};

            Runnable runMethod = () -> {
                if (Robot.driverController.getButton(Button.START)) {
                    Swerve.getInstance().zeroYaw();

                    Swerve.getInstance().gyroOffset = Swerve.getInstance().getGyroRotation().getDegrees();
                }
            };

            Runnable endMethod = () -> {};

            return new Action(startMethod, runMethod, endMethod, ActionConstants.WILL_NOT_CANCEL);
        }
    }

    @Override
    public void log() {
        SmartDashboard.putNumber("Gyro Yaw", this.gyro.getYaw());
        SmartDashboard.putNumber("Gyro Pitch", this.gyro.getPitch());
        SmartDashboard.putNumber("Gyro Roll", this.gyro.getRoll());
    }

    @Override
    public void periodic() {
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

        for (int i = 0; i < 4; i++) {
            swerveModuleStates[i] = Swerve.getInstance().swerveMods[i].getState();
        }
    }

    @Override
    public void queryInitialActions() {
        Robot.autonomousRunner.add(
            this.getLoggingAction(),
            this.getPeriodicAction()
        );

        Robot.teleopRunner.add(
            this.getLoggingAction(),
            this.getPeriodicAction(),
            SwerveActions.defaultDriveAction(),
            SwerveActions.startButtonAction()
        );
    }
}
