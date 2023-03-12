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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
    
    private ProfiledPIDController rotationController = new ProfiledPIDController(
        DriveConstants.DRIVE_ROTATION_KP,
        DriveConstants.DRIVE_ROTATION_KI, 
        DriveConstants.DRIVE_ROTATION_KD, 
        new TrapezoidProfile.Constraints(DriveConstants.DRIVE_MAX_ROTATIONAL_VELOCITY, DriveConstants.DRIVE_MAX_ROTATIONAL_ACCELERATION)
    );

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    private boolean isSnapping = false;

    private double snapSetpoint;

    private double snapStartTime;

    public boolean slowMode;

    public double initialGyroOffset;
    
    private Swerve() {
        this.zeroGyro();

        this.rotationController.enableContinuousInput(0.0, 2.0 * Math.PI);

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

    public double getPitch() {
        return this.gyro.getPitch();
    }

    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(-this.gyro.getYaw() + 180.0);
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
        double xSpeed, ySpeed, steerSpeed;

        if (slowMode) {
            xSpeed = DriveConstants.DRIVE_MAX_VELOCITY_SLOW * translation.getX();
            ySpeed = DriveConstants.DRIVE_MAX_VELOCITY_SLOW * translation.getY();
            steerSpeed = DriveConstants.DRIVE_MAX_ROTATIONAL_VELOCITY_SLOW * rotation;
        } else {
            xSpeed = DriveConstants.DRIVE_MAX_VELOCITY * translation.getX();
            ySpeed = DriveConstants.DRIVE_MAX_VELOCITY * translation.getY();
            steerSpeed = DriveConstants.DRIVE_MAX_ROTATIONAL_VELOCITY * rotation;
        }

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

    public void snapToDirection(Translation2d translation, double rotationSetpoint, boolean fieldRelative, boolean isOpenLoop) {
        if (Math.abs(PoseEstimator.getInstance().getSwerveRotation().getRadians() - this.snapSetpoint) > 0.05 && Timer.getFPGATimestamp() - this.snapStartTime < 3.0) {
            double xSpeed, ySpeed, steerSpeed;

            if (slowMode) {
                xSpeed = DriveConstants.DRIVE_MAX_VELOCITY_SLOW * translation.getX();
                ySpeed = DriveConstants.DRIVE_MAX_VELOCITY_SLOW * translation.getY();
            } else {
                xSpeed = DriveConstants.DRIVE_MAX_VELOCITY * translation.getX();
                ySpeed = DriveConstants.DRIVE_MAX_VELOCITY * translation.getY();
            }

            xSpeed = Math.abs(xSpeed) > OIConstants.DRIVE_DEADBAND ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > OIConstants.DRIVE_DEADBAND ? ySpeed : 0.0;

            xSpeed = xLimiter.calculate(xSpeed);
            ySpeed = yLimiter.calculate(ySpeed);

            steerSpeed = rotationController.calculate(PoseEstimator.getInstance().getSwerveRotation().getRadians() % 2.0 * Math.PI, this.snapSetpoint);

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
        } else {
            this.isSnapping = false;
        }
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

    public void printModuleOffsets() {
        System.out.println("Front left: " + frontLeft.getAbsoluteEncoderRad());
        System.out.println("Front right: " + frontRight.getAbsoluteEncoderRad());
        System.out.println("Back left: " + backLeft.getAbsoluteEncoderRad());
        System.out.println("Back right: " + backRight.getAbsoluteEncoderRad());
    }

    public double getSwerveRotation() {
        double rotAxis = -Math.pow(Robot.driverController.getAxis(Side.RIGHT, Axis.X), 3);

        if (Math.abs(rotAxis) < OIConstants.DRIVE_DEADBAND) {
            return 0.0;
        } else {
            return rotAxis;
        }
    }

    public void setSnapSetpoint(double setpoint) {
        this.snapSetpoint = setpoint;
        this.snapStartTime = Timer.getFPGATimestamp();

        // this.isSnapping = true;
    }

    public double getSnapSetpoint() {
        return snapSetpoint;
    }

    public boolean isSnapping() {
        return isSnapping;
    }

    private static final class SwerveActions {
        public static final Action defaultDriveAction() {
            Runnable startMethod = () -> {
                Swerve.getInstance().drive(new Translation2d(), 0.0, true, true);
            };

            Runnable runMethod = () -> {
                if (Robot.driverController.getButton(Button.RB)) {
                    Swerve.getInstance().slowMode = true;
                } else {
                    Swerve.getInstance().slowMode = false;
                }

                // if (Robot.driverController.getButton(Button.Y)) {
                //     return 0.0;
                // } else if (Robot.driverController.getButton(Button.X)) {
                //     return 0.5 * Math.PI;
                // } else if (Robot.driverController.getButton(Button.A)) {
                //     return Math.PI;
                // } else {
                //     return 1.5 * Math.PI;
                // }

                Translation2d swerveTranslation = Swerve.getInstance().getSwerveTranslation();
                
                double swerveRotation = Swerve.getInstance().getSwerveRotation();

                if (Swerve.getInstance().isSnapping()) {
                    Swerve.getInstance().snapToDirection(swerveTranslation, Swerve.getInstance().snapSetpoint, true, true);
                } else if (Robot.driverController.getButton(Button.Y)) {
                    Swerve.getInstance().setSnapSetpoint(0);
                } else if (Robot.driverController.getButton(Button.X)) {
                    Swerve.getInstance().setSnapSetpoint(0.5 * Math.PI);
                } else if (Robot.driverController.getButton(Button.A)) {
                    Swerve.getInstance().setSnapSetpoint(Math.PI);
                } else if (Robot.driverController.getButton(Button.B)) {
                    Swerve.getInstance().setSnapSetpoint(1.5 * Math.PI);
                } else if (Robot.driverController.getButton(Button.LB)) {
                    Swerve.getInstance().drive(swerveTranslation, swerveRotation, false, true);
                } else {
                    Swerve.getInstance().drive(swerveTranslation, swerveRotation, true, true);
                }

                Swerve.getInstance().printModuleOffsets();
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
                }
            };

            Runnable endMethod = () -> {};

            return new Action(startMethod, runMethod, endMethod, ActionConstants.WILL_NOT_CANCEL);
        }
    }

    @Override
    public void log() {
        SmartDashboard.putNumber("Gyro Heading", this.getGyroRotation().getDegrees());
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
