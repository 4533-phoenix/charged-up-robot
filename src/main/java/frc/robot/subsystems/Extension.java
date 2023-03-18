package frc.robot.subsystems;

import frc.libs.java.actions.*;
import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.controls.PSController.Button;

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

public final class Extension extends Subsystem {
    private static Extension mInstance;

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
    private double startSetpoint;
    private double currTime, startTime, waitTime;
    private boolean armInWaiting = false;

    enum LowerExtensionState {
        ZERO_INCHES, FIVE_INCHES, SEVEN_INCHES, TWELVE_INCHES, OFF
    }

    public final double[] elbowSetpoints = {15.5, 15.5, 37.5, 93.0, 177.0};

    public static enum ExtensionState {
        GROUND_LOW_INTAKE, GROUND_HIGH_INTAKE, OFF_GROUND, MIDDLE_ROW, HIGH_ROW, LOWER, HIGHER
    }

    private Extension() {
        elbowMotor.setInverted(true);
        elbowMotor.setSmartCurrentLimit(30);
        elbowMotor.setClosedLoopRampRate(4.0);
    }

    public static Extension getInstance() {
        if (mInstance == null) {
            mInstance = new Extension();
        }
        
        return mInstance;
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
        this.startSetpoint = elbowController.getSetpoint();

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
                    case HIGHER:
                        this.armSetpoint += 0.002;
                        this.elbowController.setSetpoint(armSetpoint);
                        break;
                    case LOWER:
                        if (!prevState.equals(ExtensionState.HIGH_ROW) || this.getElbowAngle().getDegrees() > 180.0) {
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

    private static final class ExtensionActions {
        public static final Action defaultExtensionAction() {
            Runnable startMethod = () -> {};

            Runnable runMethod = () -> {
                if (Robot.operatorController.getButton(Button.Y)) {
                    Extension.getInstance().updateExtensionState(ExtensionState.GROUND_LOW_INTAKE);
                } else if (Robot.operatorController.getButton(Button.BACK)) {
                    Extension.getInstance().updateExtensionState(ExtensionState.GROUND_HIGH_INTAKE);
                } else if (Robot.operatorController.getButton(Button.X)) {
                    Extension.getInstance().updateExtensionState(ExtensionState.OFF_GROUND);
                } else if (Robot.operatorController.getButton(Button.B)) {
                    Extension.getInstance().updateExtensionState(ExtensionState.MIDDLE_ROW);
                } else if (Robot.operatorController.getButton(Button.A)) {
                    Extension.getInstance().updateExtensionState(ExtensionState.HIGH_ROW);
                } else if (Robot.operatorController.getButton(Button.RB)) {
                    Extension.getInstance().updateExtensionState(ExtensionState.HIGHER);
                } else if (Robot.operatorController.getButton(Button.LB)) {
                    Extension.getInstance().updateExtensionState(ExtensionState.LOWER);
                } else {
                    Extension.getInstance().updateExtensionState();
                }

                Extension.getInstance().updateElbowController();

                // System.out.println("p error: " + Extension.getInstance().getElbowController().getPositionError());
                // System.out.println("absolute: " + Extension.getInstance().getAbsoluteEncoderAbsolutePosition());
                System.out.println("angle: " + Extension.getInstance().getElbowAngle().getDegrees());
                // System.out.println("setpoint: " + Extension.getInstance().getElbowController().getSetpoint());
                // System.out.println("current: " + Extension.getInstance().elbowMotor.getOutputCurrent());
            };

            Runnable endMethod = () -> {
                Extension.getInstance().setLowerExtensionState(LowerExtensionState.OFF);
            };

            return new Action(startMethod, runMethod, endMethod, ActionConstants.WILL_NOT_CANCEL);
        }
    }

    @Override
    public void log() {
        SmartDashboard.putNumber("Upper arm angle", Extension.getInstance().getElbowAngle().getDegrees());
        SmartDashboard.putNumber("Upper arm setpoint", Extension.getInstance().armSetpoint * 360.0);
    }

    @Override
    public void periodic() {}

    @Override
    public void queryInitialActions() {
        Robot.teleopRunner.add(
            ExtensionActions.defaultExtensionAction(),
            this.getLoggingAction()
        );
    }
}