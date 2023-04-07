package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Extension implements Subsystem {
    private final DoubleSolenoid lowerExtensionCylinder = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
        ExtensionConstants.LOWER_EXTENSION_PCM_PORT_FORWARD, ExtensionConstants.LOWER_EXTENSION_PCM_PORT_REVERSE);
    private final DoubleSolenoid upperExtensionCylinder = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        ExtensionConstants.UPPER_EXTENSION_PCM_PORT_FORWARD, ExtensionConstants.UPPER_EXTENSION_PCM_PORT_REVERSE);

    private final CANSparkMax elbowMotor = new CANSparkMax(ExtensionConstants.ELBOW_MOTOR_ID, MotorType.kBrushless);
    private final PIDController elbowController = new PIDController(ExtensionConstants.ELBOW_KP, ExtensionConstants.ELBOW_KI,
        ExtensionConstants.ELBOW_KD);
    private final ArmFeedforward elbowFeedforward = new ArmFeedforward(ExtensionConstants.ELBOW_KS, 
        ExtensionConstants.ELBOW_KG, ExtensionConstants.ELBOW_KV, ExtensionConstants.ELBOW_KA);

    public final DutyCycleEncoder elbowAbsoluteEncoder = new DutyCycleEncoder(ExtensionConstants.ELBOW_ABSOLUTE_ENCODER_PORT);
    public final Encoder elbowRelativeEncoder = new Encoder(ExtensionConstants.ELBOW_ENCODER_PORT_A, ExtensionConstants.ELBOW_ENCODER_PORT_B);

    public double initialAbsoluteEncoderPosition;

    private double armSetpoint;

    private ExtensionState prevState = ExtensionState.OFF_GROUND;
    private double currTime, startTime, waitTime;
    private boolean armInWaiting = false;

    enum LowerExtensionState {
        ZERO_INCHES, FIVE_INCHES, SEVEN_INCHES, TWELVE_INCHES, OFF
    }

    public final double[] elbowSetpoints = {15.5, 15.5, 37.5, 103.0, 177.0, 106.0, 145.0};

    public static enum ExtensionState {
        GROUND_LOW_INTAKE, GROUND_HIGH_INTAKE, OFF_GROUND, MIDDLE_ROW, HIGH_ROW, ABOVE_MATCH_START, KNOCKDOWN, LOWER, HIGHER
    }

    public Extension() {
        elbowMotor.setInverted(true);
        elbowMotor.setSmartCurrentLimit(30);
        elbowMotor.setClosedLoopRampRate(4.0);
    }

    public void setLowerExtensionState(LowerExtensionState state) {
        switch (state) {
            case ZERO_INCHES:
                lowerExtensionCylinder.set(Value.kReverse);
                upperExtensionCylinder.set(Value.kReverse);
                break;
            case FIVE_INCHES:
                lowerExtensionCylinder.set(Value.kReverse);
                upperExtensionCylinder.set(Value.kForward);
                break;
            case SEVEN_INCHES:
                lowerExtensionCylinder.set(Value.kForward);
                upperExtensionCylinder.set(Value.kReverse);
                break;
            case TWELVE_INCHES:
                lowerExtensionCylinder.set(Value.kForward);
                upperExtensionCylinder.set(Value.kForward);
                break;
            case OFF:
                lowerExtensionCylinder.set(Value.kOff);
                upperExtensionCylinder.set(Value.kOff);
                break;
        }
    }

    public void updateExtensionState() {
        if (armInWaiting) {
            updateExtensionState(prevState);
        }
    }

    public void updateExtensionState(ExtensionState state) {
        this.currTime = Timer.getFPGATimestamp();

        if (armInWaiting && currTime - startTime > waitTime) {
            armInWaiting = false;
        }

        if (!armInWaiting) {
            if (prevState.equals(ExtensionState.HIGH_ROW) && state.equals(ExtensionState.MIDDLE_ROW)) {
                this.setLowerExtensionState(LowerExtensionState.FIVE_INCHES);
                this.startTime = currTime;
                this.waitTime = 0.4;
                this.armInWaiting = true;
            } else if (prevState.equals(ExtensionState.HIGH_ROW) && (state.equals(ExtensionState.GROUND_LOW_INTAKE) || state.equals(ExtensionState.GROUND_HIGH_INTAKE) || state.equals(ExtensionState.OFF_GROUND))) {
                this.setLowerExtensionState(LowerExtensionState.ZERO_INCHES);
                this.startTime = currTime;
                this.waitTime = 0.8;
                this.armInWaiting = true;
            } else if (prevState.equals(ExtensionState.MIDDLE_ROW) && (state.equals(ExtensionState.GROUND_LOW_INTAKE) || state.equals(ExtensionState.GROUND_HIGH_INTAKE) || state.equals(ExtensionState.OFF_GROUND))) {
                this.setLowerExtensionState(LowerExtensionState.ZERO_INCHES);
                this.startTime = currTime;
                this.waitTime = 0.4;
                this.armInWaiting = true;
            } else if (prevState.equals(ExtensionState.HIGH_ROW) && state.equals(ExtensionState.MIDDLE_ROW)) {
                this.setLowerExtensionState(LowerExtensionState.ZERO_INCHES);
                this.startTime = currTime;
                this.waitTime = 0.7;
                this.armInWaiting = true;
            } else if (prevState.equals(ExtensionState.ABOVE_MATCH_START) && state.equals(ExtensionState.HIGH_ROW)) {
                this.setLowerExtensionState(LowerExtensionState.TWELVE_INCHES);
                this.startTime = currTime;
                this.waitTime = 0.25;
                this.armInWaiting = true;
            } else if (!prevState.equals(ExtensionState.KNOCKDOWN) && state.equals(ExtensionState.KNOCKDOWN)) {
                this.startTime = currTime;
                this.waitTime = 2.0;
                this.armInWaiting = true;
            } else {
                switch (state) {
                    case GROUND_LOW_INTAKE:
                        this.setLowerExtensionState(LowerExtensionState.ZERO_INCHES);
                        this.elbowController.setSetpoint(elbowSetpoints[0] / 360.0);
                        break;
                    case GROUND_HIGH_INTAKE:
                        this.setLowerExtensionState(LowerExtensionState.ZERO_INCHES);
                        this.elbowController.setSetpoint(elbowSetpoints[1] / 360.0);
                        break;
                    case OFF_GROUND:
                        this.setLowerExtensionState(LowerExtensionState.ZERO_INCHES);
                        this.elbowController.setSetpoint(elbowSetpoints[2] / 360.0);
                        break;
                    case MIDDLE_ROW:
                        this.setLowerExtensionState(LowerExtensionState.FIVE_INCHES);
                        this.elbowController.setSetpoint(elbowSetpoints[3] / 360.0);
                        break;
                    case HIGH_ROW:
                        this.setLowerExtensionState(LowerExtensionState.TWELVE_INCHES);
                        this.elbowController.setSetpoint(elbowSetpoints[4] / 360.0);
                        break;
                    case ABOVE_MATCH_START:
                        this.setLowerExtensionState(LowerExtensionState.ZERO_INCHES);
                        this.elbowController.setSetpoint(elbowSetpoints[5] / 360.0);
                        break;
                    case KNOCKDOWN:
                        this.setLowerExtensionState(LowerExtensionState.ZERO_INCHES);
                        this.elbowController.setSetpoint(elbowSetpoints[6] / 360.0);
                        break;
                    case HIGHER:
                        this.armSetpoint += 0.002;
                        this.elbowController.setSetpoint(armSetpoint);
                        break;
                    case LOWER:
                        if (!prevState.equals(ExtensionState.HIGH_ROW) || this.getElbowAngle().getDegrees() > 183.0) {
                            this.armSetpoint -= 0.002;
                            this.elbowController.setSetpoint(armSetpoint);
                        }
                        break;
                }
            }
        }

        if (!state.equals(ExtensionState.HIGHER) && !state.equals(ExtensionState.LOWER)) {
            this.prevState = state;
        }
    }

    public double getAbsoluteEncoderAbsolutePosition() {
        return elbowAbsoluteEncoder.getAbsolutePosition();
    }

    public double getRelativeEncoderPosition() {
        return (double) elbowRelativeEncoder.get() / 2048.0 + initialAbsoluteEncoderPosition;
    }

    public Rotation2d getElbowAngle() {
        return new Rotation2d(getRelativeEncoderPosition() * 2.0 * Math.PI * ExtensionConstants.ELBOW_CHAIN_GEAR_RATIO);
    }

    public void updateElbowController() {
        double pid = elbowController.calculate(getElbowAngle().getRotations());
        double feedForward = getElbowFeedforward();
        elbowMotor.set(pid + feedForward);

        armSetpoint = elbowController.getSetpoint();
    }

    public double getElbowFeedforward() {
        double feedForward = elbowFeedforward.calculate(armSetpoint + (34.4 / 360.0), 0);
        return feedForward;
    }

    public PIDController getElbowController() {
        return this.elbowController;
    }

    @Override
    public void periodic() {
        // System.out.println("p error: " + Extension.getInstance().getElbowController().getPositionError());
        // System.out.println("absolute: " + Extension.getInstance().getAbsoluteEncoderAbsolutePosition());
        // System.out.println("angle: " + Extension.getInstance().getElbowAngle().getDegrees());
        // System.out.println("setpoint: " + Extension.getInstance().getElbowController().getSetpoint());
        // System.out.println("current: " + Extension.getInstance().elbowMotor.getOutputCurrent());

        SmartDashboard.putNumber("Upper arm angle", this.getElbowAngle().getDegrees());
        SmartDashboard.putNumber("Upper arm setpoint", this.armSetpoint * 360.0);
    }
}