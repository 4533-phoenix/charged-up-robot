package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;

public final class SwerveModule {
    private final CANSparkMax driveMotor;

    private final CANSparkMax steerMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;

    private final PIDController steerPIDController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    /**
     * Constructor for the SwerveModule class. Initializes the motor controllers, encoders, and controllers
     * for each swerve module and sets any necessary conditions (inversions, etc.)
     * 
     * @param driveMotorID The CAN ID of the drive motor
     * @param steerMotorID The CAN ID of the steer motor
     * @param driveMotorReversed Inversion status of the drive motor controller
     * @param steerMotorReversed Inversion status of the steer motor controller
     * @param absoluteEncoderID The CAN ID of the absolute encoder
     * @param absoluteEncoderOffset The offset of the absolute encoder from zero
     * @param absoluteEncoderReversed Inversion status of the absolute encoder
     */
    public SwerveModule(int driveMotorID, int steerMotorID, boolean driveMotorReversed, boolean steerMotorReversed,
            int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoder = new CANCoder(absoluteEncoderID);
        
        this.driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        this.steerMotor = new CANSparkMax(steerMotorID, MotorType.kBrushless);

        this.driveMotor.setInverted(driveMotorReversed);
        this.steerMotor.setInverted(steerMotorReversed);

        this.driveMotor.setIdleMode(IdleMode.kBrake);

        this.driveMotor.setSmartCurrentLimit(30);
        this.steerMotor.setSmartCurrentLimit(30);

        this.driveEncoder = this.driveMotor.getEncoder();
        this.steerEncoder = this.steerMotor.getEncoder();

        this.driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_METERS_PER_ROTATION);
        this.driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_METERS_PER_SECOND);
        this.steerEncoder.setPositionConversionFactor(ModuleConstants.STEER_ENCODER_RADIANS_PER_ROTATION);
        this.steerEncoder.setVelocityConversionFactor(ModuleConstants.STEER_ENCODER_RADIANS_PER_SECOND);

        this.steerPIDController = new PIDController(
            ModuleConstants.STEER_KP, 
            ModuleConstants.STEER_KI, 
            ModuleConstants.STEER_KD
        );
        this.steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

        this.resetEncoders();
    }

    /**
     * @return Position traveled by the drive encoder in meters
     */
    public double getDrivePosition() {
        return this.driveEncoder.getPosition();
    }

    /**
     * @return Position traveled by the steer encoder in radians
     */
    public double getSteerPosition() {
        return this.steerEncoder.getPosition();
    }

    /**
     * @return Velocity of the drive encoder in meters per second
     */
    public double getDriveVelocity() {
        return this.driveEncoder.getVelocity();
    }

    /**
     * @return Velocity of the steer encoder in radians per second
     */
    public double getSteerVelocity() {
        return this.steerEncoder.getVelocity();
    }

    /**
     * Gives the position of the steer absolute encoder in radians, on a scale of 0 to 6.28
     * 
     * @return Position of the absolute encoder in radians
     */
    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition();
        angle *= Math.PI / 180.0;
        angle %= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;

        if (angle < 0.0)
            angle += 2.0 * Math.PI;

        return angle * (this.absoluteEncoderReversed ? -1.0 : 1.0);
    }

    /**
     * Gives the absolute reading of where the absolute encoder thinks it is in radians
     * Used for reading the absolute encoder offset
     * 
     * @return Absolute position of the absolute encoder in radians
     */
    public double getAbsoluteEncoderValue() {
        return absoluteEncoder.getAbsolutePosition() * (Math.PI / 180.00) * (this.absoluteEncoderReversed ? -1.0 : 1.0);
    }

    /**
     * Sets the drive encoder to position = 0 and the steer relative encoder to the position of the 
     * steer absolute encoder
     */
    public void resetEncoders() {
        this.driveEncoder.setPosition(0.0);
        this.steerEncoder.setPosition(this.getAbsoluteEncoderRad());
    }

    /**
     * Gives the position of the drive and steer motors
     * 
     * @return An object containing the drive and steer motor positions
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(this.getDrivePosition(), new Rotation2d(this.getSteerPosition()));
    }

    /**
     * Gives the state of the swerve module (drive velocity and steer position)
     * 
     * @return An object containing the drive motor velocity and the steer motor position
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(this.getDriveVelocity(), new Rotation2d(this.getSteerPosition()));
    }

    /**
     * Optimizes and sets the speeds of the drive and steer motors
     * 
     * @param state The state of the swerve module
     */
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            this.stop();
            return;
        }

        state = SwerveModuleState.optimize(state, this.getState().angle);

        this.driveMotor.set(state.speedMetersPerSecond / DriveConstants.DRIVE_MAX_PHYSICAL_VELOCITY);
        this.steerMotor.set(this.steerPIDController.calculate(this.getSteerPosition(), state.angle.getRadians()));
    }

    /**
     * Sets the speeds of the drive and steer motors to zero
     */
    public void stop() {
        this.driveMotor.set(0.0);
        this.steerMotor.set(0.0);
    }
}