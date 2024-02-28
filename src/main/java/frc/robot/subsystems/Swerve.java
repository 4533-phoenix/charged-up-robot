package frc.robot.subsystems;

import frc.robot.controls.PSController.Axis;
import frc.robot.controls.PSController.Button;
import frc.robot.controls.PSController.Side;
import frc.robot.helpers.LimelightHelper;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
//import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.path.PathPlannerTrajectory.State;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

// import com.pathplanner.lib.util.ReplanningConfig;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
//import com.pathplanner.lib.commands.SwerveControllerCommand;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand; 

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.math.MatBuilder;
// import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.math.trajectory.Trajectory.State;

public final class Swerve implements Subsystem {
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
    
    // private PIDController xController = new PIDController(AutoConstants.AUTO_X_VELOCITY_KP, 
    //     AutoConstants.AUTO_X_VELOCITY_KI, AutoConstants.AUTO_X_VELOCITY_KD);
    // private PIDController yController = new PIDController(AutoConstants.AUTO_Y_VELOCITY_KP, 
    //     AutoConstants.AUTO_Y_VELOCITY_KI, AutoConstants.AUTO_Y_VELOCITY_KD);
    private PIDController thetaController = new PIDController(AutoConstants.AUTO_ROTATION_KP, 
        AutoConstants.AUTO_ROTATION_KI, AutoConstants.AUTO_ROTATION_KD);

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    private DriveSpeed driveSpeed;

    private Field2d mField2d = new Field2d();

    public enum DriveSpeed {
        SLOW, STANDARD, FAST
    }

    public SwerveDrivePoseEstimator swervePoseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.SWERVE_KINEMATICS, 
        this.getGyroRotation(),
        this.getModulePositions(),
        new Pose2d()
    );
    
    public Swerve() {
        this.zeroGyro();

        this.thetaController.enableContinuousInput(-Math.PI, Math.PI);

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

    public void zeroYaw() {
        this.gyro.zeroYaw();
    }

    public double getPitch() {
        return this.gyro.getPitch();
    }

    public Rotation2d getGyroRotation() {
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

    public void setDriveSpeed(DriveSpeed speed) {
        this.driveSpeed = speed;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        double xSpeed, ySpeed, steerSpeed;

        switch (this.driveSpeed) {
            case SLOW:
                xSpeed = DriveConstants.DRIVE_MAX_VELOCITY_SLOW * translation.getX();
                ySpeed = DriveConstants.DRIVE_MAX_VELOCITY_SLOW * translation.getY();
                steerSpeed = DriveConstants.DRIVE_MAX_ROTATIONAL_VELOCITY_SLOW * rotation;
                break;
            case STANDARD:
                xSpeed = DriveConstants.DRIVE_MAX_VELOCITY * translation.getX();
                ySpeed = DriveConstants.DRIVE_MAX_VELOCITY * translation.getY();
                steerSpeed = DriveConstants.DRIVE_MAX_ROTATIONAL_VELOCITY * rotation;
                break;
            case FAST:
                xSpeed = DriveConstants.DRIVE_MAX_VELOCITY_FAST * translation.getX();
                ySpeed = DriveConstants.DRIVE_MAX_VELOCITY_FAST * translation.getY();
                steerSpeed = DriveConstants.DRIVE_MAX_ROTATIONAL_VELOCITY_FAST * rotation;
                break;
            default:
                xSpeed = DriveConstants.DRIVE_MAX_VELOCITY * translation.getX();
                ySpeed = DriveConstants.DRIVE_MAX_VELOCITY * translation.getY();
                steerSpeed = DriveConstants.DRIVE_MAX_ROTATIONAL_VELOCITY * rotation;
                break;
        }

        xSpeed = Math.abs(xSpeed) > OIConstants.DRIVE_DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.DRIVE_DEADBAND ? ySpeed : 0.0;
        steerSpeed = Math.abs(steerSpeed) > OIConstants.DRIVE_DEADBAND ? steerSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        steerSpeed = steerLimiter.calculate(steerSpeed);

        ChassisSpeeds chassisSpeeds;

        if (fieldRelative) {
            if (Robot.operatorController.getButton(Button.BACK)) {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, steerSpeed, getGyroRotation()
                );
            } else {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, steerSpeed, this.getEstimatedPose().getRotation()
                );
            }
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

    public Pose2d getEstimatedPose() {
        return this.swervePoseEstimator.getEstimatedPosition();
    }

    public void resetSwervePose() {
        this.swervePoseEstimator.resetPosition(this.getEstimatedPose().getRotation(), this.getModulePositions(), new Pose2d());
    }

    public void resetPoseEstimator(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d startPose) {
        this.swervePoseEstimator.resetPosition(gyroAngle, modulePositions, startPose);
    }

    public void stopDrive() {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

        this.setModuleStates(DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds));
    }

    public void addVisionPose2d() {
        for (String limelightName : LimelightConstants.LIMELIGHT_NAMES) {
            if (!LimelightHelper.getTV(limelightName)) {
                continue;
            }

            Pose2d pose;

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()  && alliance.get() == DriverStation.Alliance.Red) {
                pose = LimelightHelper.getBotPose2d_wpiRed(limelightName);
            } else {
                pose = LimelightHelper.getBotPose2d_wpiBlue(limelightName);
            }

           // double trust = ((1 - LimelightHelper.getTA(limelightName)) * Constants.PoseEstimatorConstants.VISION_UNTRUST);
            double latency = LimelightHelper.getLatency_Pipeline(limelightName) / 1000.0;

            // BROKEN, JEC 2024 Matbuilder deprecated
            //swervePoseEstimator.setVisionMeasurementStdDevs(new MatBuilder<>(Nat.N3(), Nat.N1()).fill(trust, trust, trust));
            swervePoseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - latency);
        }
    }

    public void updatePoseEstimator() {
        swervePoseEstimator.update(this.getGyroRotation(), this.getModulePositions());
        addVisionPose2d();

        if (DriverStation.getAlliance().equals(Alliance.Red)) {
            Pose2d estimatedPose = this.getEstimatedPose();
            mField2d.setRobotPose(new Pose2d(16.53 - estimatedPose.getX(), 8.02 - estimatedPose.getY(), estimatedPose.getRotation().plus(Rotation2d.fromDegrees(180))));
        } else {
            mField2d.setRobotPose(this.getEstimatedPose());
        }
    }
    
    public void driveDistance(double distanceMeters) {
        double startTime = Timer.getFPGATimestamp();
        double totalTime = Math.abs(distanceMeters) / AutoConstants.AUTO_MAX_VELOCITY;

        ChassisSpeeds driveSpeeds;
        
        if (distanceMeters >= 0) {
            driveSpeeds =  new ChassisSpeeds(AutoConstants.AUTO_MAX_VELOCITY, 0, 0);
        } else {
            driveSpeeds = new ChassisSpeeds(-AutoConstants.AUTO_MAX_VELOCITY, 0, 0);
        }

        SwerveModuleState[] moduleStates;

        while (Timer.getFPGATimestamp() - startTime <= totalTime) {
            moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(driveSpeeds);

            this.setModuleStates(moduleStates);
        }

        this.stopDrive();
    }

    public void printModuleOffsets() {
        System.out.println("Front left: " + frontLeft.getAbsoluteEncoderValue());
        System.out.println("Front right: " + frontRight.getAbsoluteEncoderValue());
        System.out.println("Back left: " + backLeft.getAbsoluteEncoderValue());
        System.out.println("Back right: " + backRight.getAbsoluteEncoderValue());
    }

    public void enableChargeStation(boolean direction) {
        ChassisSpeeds driveSpeeds;
       
        if (direction) {
            driveSpeeds =  new ChassisSpeeds(Units.feetToMeters(6.5), 0, 0);
        } else {
            driveSpeeds = new ChassisSpeeds(Units.feetToMeters(-6.5), 0, 0);
        }

        SwerveModuleState[] moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(driveSpeeds);

        this.setModuleStates(moduleStates);

        while (Math.abs(this.getPitch()) < 8.0) {
            moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(driveSpeeds);

            this.setModuleStates(moduleStates);
        }

        if (direction) {
            driveSpeeds =  new ChassisSpeeds(Units.feetToMeters(2.0), 0, 0);
        } else {
            driveSpeeds = new ChassisSpeeds(Units.feetToMeters(-2.0), 0, 0);
        }

        while (Math.abs(this.getPitch()) > 2.0) {
            moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(driveSpeeds);

            this.setModuleStates(moduleStates);
        }
        
        this.setModuleStates(DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds()));
    }

    public void straightenWheels(double time) {
        double startTime = Timer.getFPGATimestamp();

        ChassisSpeeds driveSpeeds;

        driveSpeeds =  new ChassisSpeeds(Units.feetToMeters(-0.25), 0, 0);

        while (Timer.getFPGATimestamp() - startTime < time) {
            SwerveModuleState[] moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(driveSpeeds);

            this.setModuleStates(moduleStates);
        }

        this.setModuleStates(DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds()));
    }

    public void driveOverChargeStation(boolean direction) {
        ChassisSpeeds driveSpeeds;
       
        if (direction) {
            driveSpeeds =  new ChassisSpeeds(Units.feetToMeters(8.0), 0, 0);
        } else {
            driveSpeeds = new ChassisSpeeds(Units.feetToMeters(-8.0), 0, 0);
        }

        SwerveModuleState[] moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(driveSpeeds);

        this.setModuleStates(moduleStates);

        while (Math.abs(this.getPitch()) < 8.0) {}

        while (Math.abs(this.getPitch()) > 2.0) {}

        while (Math.abs(this.getPitch()) < 8.0) {}

        while (Math.abs(this.getPitch()) > 1.5) {}
        
        this.setModuleStates(DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds()));
    }

    public void adjustChargeStation() {
        while (Math.abs(this.getPitch()) > DriveConstants.CHARGE_STATION_PITCH_DEADBAND) {
            ChassisSpeeds driveSpeeds = this.getPitch() < 0.0 ? new ChassisSpeeds(-0.3, 0.0, 0.0) : new ChassisSpeeds(0.3, 0.0, 0.0);

            this.setModuleStates(DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(driveSpeeds));
        }

        this.setModuleStates(DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds()));
    }

    // public Command followTrajectoryCommand(PathPlannerTrajectory path) {
    //     return new SwerveControllerCommand(   //was PPSwerveControllerCommand  but JEC changed it
    //         path, 
    //         this::getEstimatedPose, 
    //         DriveConstants.SWERVE_KINEMATICS, 
    //         this.xController, 
    //         this.yController, 
    //         this.thetaController, 
    //         this::setModuleStates,
    //         true,
    //         this).andThen(() -> stopDrive());
    // }

    @Override
    public void periodic() {
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

        for (int i = 0; i < 4; i++) {
            swerveModuleStates[i] = this.swerveMods[i].getState();
        }

        this.updatePoseEstimator();

        SmartDashboard.putNumber("Gyro Heading", this.getGyroRotation().getDegrees());
        SmartDashboard.putNumber("Gyro Pitch", this.gyro.getPitch());

        SmartDashboard.putNumber("Robot Pose - X", this.getEstimatedPose().getX());
        SmartDashboard.putNumber("Robot Pose - Y", getEstimatedPose().getY());
        SmartDashboard.putNumber("Robot Pose - Angle", getEstimatedPose().getRotation().getDegrees());

        SmartDashboard.putData("Field", mField2d);

        this.printModuleOffsets();
    }
}
