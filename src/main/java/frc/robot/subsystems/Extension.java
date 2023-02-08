package frc.robot.subsystems;

import frc.libs.java.actions.*;
import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.controls.PSController.Button;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Extension extends Subsystem {
    public static Extension mInstance;

    private Extension() {
        elbowMotor.setInverted(true);
    }

    private final DoubleSolenoid lowerExtensionCylinder = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
        ExtensionConstants.LOWER_EXTENSION_PCM_PORT_FORWARD, ExtensionConstants.LOWER_EXTENSION_PCM_PORT_REVERSE);
    private final DoubleSolenoid upperExtensionCylinder = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        ExtensionConstants.UPPER_EXTENSION_PCM_PORT_FORWARD, ExtensionConstants.UPPER_EXTENSION_PCM_PORT_REVERSE);

    private final CANSparkMax elbowMotor = new CANSparkMax(ExtensionConstants.ELBOW_MOTOR_ID, MotorType.kBrushless);
    private final PIDController elbowController = new PIDController(ExtensionConstants.ELBOW_KP, 
        ExtensionConstants.ELBOW_KI, ExtensionConstants.ELBOW_KD);
    private final ArmFeedforward elbowFeedforward = new ArmFeedforward(ExtensionConstants.ELBOW_KS, 
        ExtensionConstants.ELBOW_KG, ExtensionConstants.ELBOW_KV, ExtensionConstants.ELBOW_KA);

    private final DutyCycleEncoder elbowAbsoluteEncoder = new DutyCycleEncoder(ExtensionConstants.ELBOW_ABSOLUTE_ENCODER_PORT);
    private final Encoder elbowRelativeEncoder = new Encoder(ExtensionConstants.ELBOW_ENCODER_PORT_A, ExtensionConstants.ELBOW_ENCODER_PORT_B);

    public double initialAbsoluteEncoderPosition;

    enum LowerExtensionState {
        ZERO_INCHES, FIVE_INCHES, SEVEN_INCHES, TWELVE_INCHES, OFF
    }

    private final double[] elbowSetpoints = {10.0, 75.0, 130.0, 100.0};

    enum ExtensionState {
        GROUND_INTAKE, MIDDLE_ROW, HIGH_ROW, SUBSTATION
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

    public void setUpperExtensionState(boolean state) {
        if (state) {
            elbowController.setSetpoint(elbowSetpoints[1]);
        } else {
            elbowController.setSetpoint(elbowSetpoints[0]);
        }
    }

    public void setExtensionState(ExtensionState state) {
        switch (state) {
            case GROUND_INTAKE:
                this.setLowerExtensionState(LowerExtensionState.ZERO_INCHES);
                //this.elbowController.setSetpoint(elbowSetpoints[0]);
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
        return (double) elbowRelativeEncoder.get() / 2048.0 + initialAbsoluteEncoderPosition;
    }

    public Rotation2d getElbowAngle() {
        return new Rotation2d(getRelativeEncoderPosition() * 2.0 * Math.PI * ExtensionConstants.ELBOW_CHAIN_GEAR_RATIO);
    }

    public void updateElbowController() {
        double elbowPosition = getElbowAngle();

        elbowMotor.set(elbowController.calculate(elbowPosition));
    }

    public void updateElbowFeedforward() {
        elbowFeedforward.calculate(initialAbsoluteEncoderPosition, initialAbsoluteEncoderPosition)
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

    public double getArmVelocityRad() {
        return elbowMotor.get() * 2.0 * Math.PI / 117.3;
    }

    public PIDController getElbowController() {
        return this.elbowController;
    }

    private static final class ExtensionActions {
        public static final Action defaultExtensionAction() {
            Runnable startMethod = () -> {
                Extension.getInstance().elbowAbsoluteEncoder.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0);
                Extension.getInstance().elbowAbsoluteEncoder.setPositionOffset(ExtensionConstants.ELBOW_ABSOLUTE_ENCODER_OFFSET);
                Extension.getInstance().elbowRelativeEncoder.reset();
                Extension.getInstance().initialAbsoluteEncoderPosition = Extension.getInstance().getAbsoluteEncoderPosition();
                Extension.getInstance().updateElbowController();
                Extension.getInstance().setUpperExtensionState(false);
            };

            Runnable runMethod = () -> {
                if (Robot.driverController.getButton(Button.A)) {
                    //Extension.getInstance().upperExtensionUp();
                    Extension.getInstance().setUpperExtensionState(true);
                } else if (Robot.driverController.getButton(Button.B)) {
                    //Extension.getInstance().upperExtensionDown();
                    Extension.getInstance().setUpperExtensionState(false);
                } else {
                    //Extension.getInstance().upperExtensionStop();
                }

                if (Robot.operatorController.getButton(Button.Y)) {
                    Extension.getInstance().setLowerExtensionState(LowerExtensionState.ZERO_INCHES);
                } else if (Robot.operatorController.getButton(Button.X)) {
                    Extension.getInstance().setLowerExtensionState(LowerExtensionState.FIVE_INCHES);
                } else if (Robot.operatorController.getButton(Button.B)) {
                    Extension.getInstance().setLowerExtensionState(LowerExtensionState.SEVEN_INCHES);
                } else if (Robot.operatorController.getButton(Button.A)) {
                    Extension.getInstance().setLowerExtensionState(LowerExtensionState.TWELVE_INCHES);
                }

                Extension.getInstance().updateElbowController();

                //System.out.println("absolute: " + Extension.getInstance().getAbsoluteEncoderPosition());
                //System.out.println("relative: " + Extension.getInstance().getRelativeEncoderPosition());
                System.out.println("angle: " + Extension.getInstance().getElbowAngle());
                //System.out.println(Extension.getInstance().getElbowController().getSetpoint());
            };

            Runnable endMethod = () -> {
                Extension.getInstance().setLowerExtensionState(LowerExtensionState.OFF);
            };

            return new Action(startMethod, runMethod, endMethod, false).withSubsystem(Extension.getInstance());
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