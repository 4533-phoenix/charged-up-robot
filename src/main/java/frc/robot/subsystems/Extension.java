package frc.robot.subsystems;

import frc.libs.java.actions.*;
import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.controls.PSController.Button;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.ArmFeedforward;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public final class Extension extends Subsystem {
    private static Extension mInstance;

    private final DoubleSolenoid lowerExtensionCylinder = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
        ExtensionConstants.LOWER_EXTENSION_PCM_PORT_FORWARD, ExtensionConstants.LOWER_EXTENSION_PCM_PORT_REVERSE);

    private final DoubleSolenoid upperExtensionCylinder = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        ExtensionConstants.UPPER_EXTENSION_PCM_PORT_FORWARD, ExtensionConstants.UPPER_EXTENSION_PCM_PORT_REVERSE);

    private final CANSparkMax elbowMotor = new CANSparkMax(ExtensionConstants.ELBOW_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxPIDController elbowController = elbowMotor.getPIDController();
    private final ArmFeedforward elbowFeedforward = new ArmFeedforward(ExtensionConstants.ELBOW_KS, 
        ExtensionConstants.ELBOW_KG, ExtensionConstants.ELBOW_KV, ExtensionConstants.ELBOW_KA);

    private final DutyCycleEncoder elbowAbsoluteEncoder = new DutyCycleEncoder(ExtensionConstants.ELBOW_ABSOLUTE_ENCODER_PORT);
    private final RelativeEncoder elbowRelativeEncoder = elbowMotor.getEncoder();

    public double initialAbsoluteEncoderPosition;

    private double armSetpoint;

    enum LowerExtensionState {
        ZERO_INCHES, FIVE_INCHES, SEVEN_INCHES, TWELVE_INCHES, OFF
    }

    public final double[] elbowSetpoints = {10.0, 75.0, 130.0, 100.0};

    public static enum ExtensionState {
        GROUND_INTAKE, MIDDLE_ROW, HIGH_ROW, SUBSTATION
    }

    private Extension() {
        elbowMotor.setInverted(true);
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

    // public void setUpperExtensionState(boolean state) {
    //     if (state) {
    //         elbowController.setSetpoint(elbowSetpoints[1]);
    //     } else {
    //         elbowController.setSetpoint(elbowSetpoints[0]);
    //     }
    // }

    public void setExtensionState(ExtensionState state) {
        switch (state) {
            case GROUND_INTAKE:
                this.setLowerExtensionState(LowerExtensionState.ZERO_INCHES);
                // this.elbowController.setSetpoint(elbowSetpoints[0]);
                break;
            case MIDDLE_ROW:
                this.setLowerExtensionState(LowerExtensionState.FIVE_INCHES);
                //this.elbowController.setSetpoint(elbowSetpoints[1]);
                break;
            case HIGH_ROW:
                this.setLowerExtensionState(LowerExtensionState.TWELVE_INCHES);
                //this.elbowController.setSetpoint(elbowSetpoints[2]);
                break;
            case SUBSTATION:
                this.setLowerExtensionState(LowerExtensionState.SEVEN_INCHES);
                //this.elbowController.setSetpoint(elbowSetpoints[3]);
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
        return (double) elbowRelativeEncoder.getPosition() / (42.0 * 44.0);
    }

    public Rotation2d getElbowAngle() {
        return new Rotation2d(getRelativeEncoderPosition() * 2.0 * Math.PI * ExtensionConstants.ELBOW_CHAIN_GEAR_RATIO);
    }

    public void updateElbowController(double degrees) {
        double positionRot = degrees / 360.0;
        armSetpoint = positionRot;

        elbowController.setReference(positionRot, ControlType.kPosition, 0, getElbowFeedforward());
    }

    public void updateElbowController() {
        elbowController.setReference(armSetpoint, ControlType.kPosition, 0, getElbowFeedforward());
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

    public SparkMaxPIDController getElbowController() {
        return this.elbowController;
    }

    private static final class ExtensionActions {
        public static final Action defaultExtensionAction() {
            Runnable startMethod = () -> {
                Extension.getInstance().elbowAbsoluteEncoder.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0);
                Extension.getInstance().elbowAbsoluteEncoder.setPositionOffset(ExtensionConstants.ELBOW_ABSOLUTE_ENCODER_OFFSET);
                Extension.getInstance().elbowRelativeEncoder.setPosition(Extension.getInstance().elbowAbsoluteEncoder.get());
                Extension.getInstance().configureElbowController();
                Extension.getInstance().updateElbowController(34.4);
            };

            Runnable runMethod = () -> {
                if (Robot.driverController.getButton(Button.A)) {
                    Extension.getInstance().updateElbowController(Extension.getInstance().elbowSetpoints[0]);
                } else if (Robot.driverController.getButton(Button.B)) {
                    Extension.getInstance().updateElbowController(Extension.getInstance().elbowSetpoints[1]);
                } else {
                    Extension.getInstance().updateElbowController();
                }

                // if (Robot.operatorController.getButton(Button.Y)) {
                //     Extension.getInstance().setLowerExtensionState(LowerExtensionState.ZERO_INCHES);
                // } else if (Robot.operatorController.getButton(Button.X)) {
                //     Extension.getInstance().setLowerExtensionState(LowerExtensionState.FIVE_INCHES);
                // } else if (Robot.operatorController.getButton(Button.B)) {
                //     Extension.getInstance().setLowerExtensionState(LowerExtensionState.SEVEN_INCHES);
                // } else if (Robot.operatorController.getButton(Button.A)) {
                //     Extension.getInstance().setLowerExtensionState(LowerExtensionState.TWELVE_INCHES);
                // }

                // Extension.getInstance().updateElbowController();

                //System.out.println("absolute: " + Extension.getInstance().getAbsoluteEncoderPosition());
                //System.out.println("relative: " + Extension.getInstance().getRelativeEncoderPosition());
                System.out.println("angle: " + Extension.getInstance().getElbowAngle().getDegrees());
                System.out.println("setpoint: " + Extension.getInstance().armSetpoint * 360.0);
                // System.out.println("p error: " + Extension.getInstance().getElbowController().getP());

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