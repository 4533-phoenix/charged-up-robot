package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;

public final class SwerveModule {
    // TODO: Make a Profiled PID Controller and a Feedforward Controller for the drive motor 
    private final CANSparkMax driveMotor;

    private final CANSparkMax steerMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;

    // TODO: Make this a Profiled PID Controller, and add a Feedforward Controller as well
    private final PIDController steerPIDController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorID, int steerMotorID, boolean driveMotorReversed, boolean steerMotorReversed,
            int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoder = new CANCoder(absoluteEncoderID);
        
        this.driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        this.steerMotor = new CANSparkMax(steerMotorID, MotorType.kBrushless);

        this.driveMotor.setInverted(driveMotorReversed);
        this.steerMotor.setInverted(steerMotorReversed);

        this.driveEncoder = this.driveMotor.getEncoder();
        this.steerEncoder = this.steerMotor.getEncoder();

        this.driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_METERS_PER_ROTATION);
        this.driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_METERS_PER_SECOND);
        this.steerEncoder.setPositionConversionFactor(ModuleConstants.STEER_ENCODER_RADIANS_PER_ROTATION);
        this.steerEncoder.setVelocityConversionFactor(ModuleConstants.STEER_ENCODER_RADIANS_PER_SECOND);

        this.steerPIDController = new PIDController(ModuleConstants.STEER_KP, 0.0, 0.0);
        this.steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

        this.resetEncoders();
    }

    public double getDrivePosition() {
        return this.driveEncoder.getPosition();
    }

    public double getSteerPosition() {
        return this.steerEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return this.driveEncoder.getVelocity();
    }

    public double getSteerVelocity() {
        return this.steerEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition();
        angle *= Math.PI / 180.0;
        angle %= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;

        if (angle < 0.0)
            angle += 2.0 * Math.PI;

        return angle * (this.absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        this.driveEncoder.setPosition(0.0);
        this.steerEncoder.setPosition(this.getAbsoluteEncoderRad());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(this.getDrivePosition(), new Rotation2d(this.getSteerPosition() * 2.0 * Math.PI));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(this.getDriveVelocity(), new Rotation2d(this.getSteerPosition() * 2.0 * Math.PI));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            this.stop();
            return;
        }

        state = SwerveModuleState.optimize(state, this.getState().angle);

        this.driveMotor.set(state.speedMetersPerSecond / DriveConstants.DRIVE_MAX_VELOCITY);
        this.steerMotor.set(this.steerPIDController.calculate(this.getSteerPosition(), state.angle.getRadians()));
    }

    public void stop() {
        this.driveMotor.set(0.0);
        this.steerMotor.set(0.0);
    }
}
