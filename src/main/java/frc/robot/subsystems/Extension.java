package frc.robot.subsystems;

import frc.libs.java.actions.*;
import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.controls.PSController.Button;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.ArmFeedforward;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

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

    private final DutyCycleEncoder elbowAbsoluteEncoder = new DutyCycleEncoder(ExtensionConstants.ELBOW_ABSOLUTE_ENCODER_PORT);
    private final Encoder elbowRelativeEncoder = new Encoder(ExtensionConstants.ELBOW_ENCODER_PORT_A, ExtensionConstants.ELBOW_ENCODER_PORT_B);
    private final AnalogInput stringPotentiometer = new AnalogInput(ExtensionConstants.STRING_POTENTIOMETER_PORT);

    public double initialAbsoluteEncoderPosition;

    private double armSetpoint;

    enum LowerExtensionState {
        ZERO_INCHES, FIVE_INCHES, SEVEN_INCHES, TWELVE_INCHES, OFF
    }

    public final double[] elbowSetpoints = {12.0, 23.5, 90.8, 170.6};

    public static enum ExtensionState {
        GROUND_INTAKE, OFF_GROUND, MIDDLE_ROW, HIGH_ROW
    }

    private Extension() {
        elbowMotor.setInverted(true);
        elbowMotor.setSmartCurrentLimit(20);
        elbowMotor.setClosedLoopRampRate(3.0);
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

    public void configureElbowController() {
        elbowController.setP(ExtensionConstants.ELBOW_KP);
        elbowController.setI(ExtensionConstants.ELBOW_KI);
        elbowController.setD(ExtensionConstants.ELBOW_KD);
    }

    public void setExtensionState(ExtensionState state) {
        if (elbowController.getSetpoint() > 160.0 / 360.0 && state.equals(ExtensionState.GROUND_INTAKE)) {
            double time = Timer.getFPGATimestamp();

            this.setLowerExtensionState(LowerExtensionState.ZERO_INCHES);

            while (Timer.getFPGATimestamp() < time + 0.6) {
                System.out.println("waiting");
            }
        }

        if (elbowController.getSetpoint() < 5.0 / 360.0 && state.equals(ExtensionState.HIGH_ROW)) {
            this.elbowController.setSetpoint(elbowSetpoints[3] / 360.0);

            while (this.getElbowAngle().getDegrees() < 90.0) {
                System.out.println("waiting");
            }
        }

        switch (state) {
            case GROUND_INTAKE:
                this.setLowerExtensionState(LowerExtensionState.ZERO_INCHES);
                this.elbowController.setSetpoint(elbowSetpoints[0] / 360.0);
                break;
            case OFF_GROUND:
                this.setLowerExtensionState(LowerExtensionState.ZERO_INCHES);
                this.elbowController.setSetpoint(elbowSetpoints[1] / 360.0);
                break;
            case MIDDLE_ROW:
                this.setLowerExtensionState(LowerExtensionState.FIVE_INCHES);
                this.elbowController.setSetpoint(elbowSetpoints[2] / 360.0);
                break;
            case HIGH_ROW:
                this.setLowerExtensionState(LowerExtensionState.TWELVE_INCHES);
                this.elbowController.setSetpoint(elbowSetpoints[3] / 360.0);
                break;
        }
    }

    public double getAbsoluteEncoderPosition() {
        return elbowAbsoluteEncoder.get();
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

    public Rotation2d getGroundToLowerArmAngle() {
        double length = getLowerExtensionLength();
        double angle = Math.acos((697.5 - Math.pow(length, 2)) / 425.0) + 19.0;
        
        return new Rotation2d(angle + 34.4);
    }

    public Rotation2d getGroundToUpperArmAngle() {
        double length = getLowerExtensionLength();
        double angle = Math.acos((697.5 - Math.pow(length, 2)) / 425.0) + 19.0;
        
        return new Rotation2d(getElbowAngle().getRadians() - angle);
    }

    public double getLowerExtensionLength() {
        return stringPotentiometer.getVoltage() * 12.0 / 5.0;
    }

    public void updateElbowController() {
        double pid = elbowController.calculate(getElbowAngle().getRotations());
        double feedForward = getElbowFeedforward();
        elbowMotor.set(pid + feedForward);
    }

    public double getElbowFeedforward() {
        double feedForward = elbowFeedforward.calculate(armSetpoint + (34.4 / 360.0), 0);
        return feedForward;
    }

    public void upperExtensionUp() {
        elbowMotor.set(-0.25);
    }

    public void upperExtensionDown() {
        elbowMotor.set(0.25);
    }

    public void upperExtensionStop() {
        elbowMotor.stopMotor();
    }

    public PIDController getElbowController() {
        return this.elbowController;
    }

    private static final class ExtensionActions {
        public static final Action defaultExtensionAction() {
            Runnable startMethod = () -> {
                Extension.getInstance().elbowAbsoluteEncoder.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0);
                Extension.getInstance().elbowAbsoluteEncoder.setPositionOffset(ExtensionConstants.ELBOW_ABSOLUTE_ENCODER_OFFSET);
                Extension.getInstance().initialAbsoluteEncoderPosition = Extension.getInstance().getAbsoluteEncoderAbsolutePosition() - Extension.getInstance().elbowAbsoluteEncoder.getPositionOffset();
                Extension.getInstance().elbowRelativeEncoder.reset();
                Extension.getInstance().configureElbowController();
                Extension.getInstance().setExtensionState(ExtensionState.GROUND_INTAKE);
            };

            Runnable runMethod = () -> {
                if (Robot.operatorController.getButton(Button.Y)) {
                    Extension.getInstance().setExtensionState(ExtensionState.GROUND_INTAKE);
                } else if (Robot.operatorController.getButton(Button.X)) {
                    Extension.getInstance().setExtensionState(ExtensionState.OFF_GROUND);
                } else if (Robot.operatorController.getButton(Button.B)) {
                    Extension.getInstance().setExtensionState(ExtensionState.MIDDLE_ROW);
                } else if (Robot.operatorController.getButton(Button.A)) {
                    Extension.getInstance().setExtensionState(ExtensionState.HIGH_ROW);
                }

                Extension.getInstance().updateElbowController();

                //  System.out.println("p error: " + Extension.getInstance().getElbowController().getPositionError());
                // System.out.println("relative: " + Extension.getInstance().getRelativeEncoderPosition());
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
    public void log() {}

    @Override
    public void periodic() {}

    @Override
    public void queryInitialActions() {
        Robot.teleopRunner.add(
            ExtensionActions.defaultExtensionAction()
        );
    }
}