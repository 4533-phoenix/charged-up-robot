package frc.robot.subsystems;

import frc.robot.Robot;
import frc.libs.java.actionLib.*;
import frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Extension extends Subsystem {
    public static Extension mInstance;

    private Extension() {}

    private final DoubleSolenoid lowerExtensionCylinder = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
        ExtensionConstants.LOWER_EXTENSION_PCM_PORT_FORWARD, ExtensionConstants.LOWER_EXTENSION_PCM_PORT_REVERSE);
    private final DoubleSolenoid upperExtensionCylinder = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        ExtensionConstants.UPPER_EXTENSION_PCM_PORT_FORWARD, ExtensionConstants.UPPER_EXTENSION_PCM_PORT_REVERSE);

    private final CANSparkMax elbowMotor = new CANSparkMax(ExtensionConstants.ELBOW_MOTOR_ID, MotorType.kBrushless);
    private final PIDController elbowController = new PIDController(ExtensionConstants.ELBOW_KP, 
        ExtensionConstants.ELBOW_KI, ExtensionConstants.ELBOW_KD);
    private final AnalogPotentiometer elbowPotentiometer = new AnalogPotentiometer(ExtensionConstants.ELBOW_POTENTIOMETER_PORT, 
        ExtensionConstants.ELBOW_POTENTIOMETER_RANGE, ExtensionConstants.ELBOW_POTENTIOMETER_OFFSET);

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
            case FIVE_INCHES:
                lowerExtensionCylinder.set(Value.kReverse);
                upperExtensionCylinder.set(Value.kForward);
            case SEVEN_INCHES:
                lowerExtensionCylinder.set(Value.kForward);
                upperExtensionCylinder.set(Value.kReverse);
            case TWELVE_INCHES:
                lowerExtensionCylinder.set(Value.kForward);
                upperExtensionCylinder.set(Value.kForward);
            case OFF:
                lowerExtensionCylinder.set(Value.kOff);
                upperExtensionCylinder.set(Value.kOff);
        }
    }

    public void setExtensionState(ExtensionState state) {
        switch (state) {
            case GROUND_INTAKE:
                this.setLowerExtensionState(LowerExtensionState.ZERO_INCHES);
                this.elbowController.setSetpoint(elbowSetpoints[0]);
            case MIDDLE_ROW:
                this.setLowerExtensionState(LowerExtensionState.FIVE_INCHES);
                this.elbowController.setSetpoint(elbowSetpoints[1]);
            case HIGH_ROW:
                this.setLowerExtensionState(LowerExtensionState.TWELVE_INCHES);
                this.elbowController.setSetpoint(elbowSetpoints[2]);
            case SUBSTATION:
                this.setLowerExtensionState(LowerExtensionState.SEVEN_INCHES);
                this.elbowController.setSetpoint(elbowSetpoints[3]);
        }
    }

    public void updateElbowController() {
        double elbowPosition = elbowPotentiometer.get();

        elbowMotor.set(elbowController.calculate(elbowPosition));
    }

    private static final class ExtensionActions {
        public static final Action defaultExtensionAction() {
            Runnable startMethod = () -> {};

            Runnable runMethod = () -> {};

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