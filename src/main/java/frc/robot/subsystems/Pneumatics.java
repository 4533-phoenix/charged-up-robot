package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.libs.java.actions.*;
import frc.robot.Robot;
import frc.robot.Constants.*;

public final class Pneumatics extends Subsystem {
    private static Pneumatics mInstance;

    private final Compressor compressor = new Compressor(GlobalConstants.PCM_ID, PneumaticsModuleType.REVPH);

    private Pneumatics() {}

    public static Pneumatics getInstance() {
        if (mInstance == null) {
            mInstance = new Pneumatics();
        }

        return mInstance;
    }

    public void enableCompressor() {
        compressor.enableDigital();
    }

    public void disableCompressor() {
        compressor.disable();
    }

    private static final class PneumaticsActions {
        public static final Action defaultPneumaticsAction() {
            Runnable startMethod = () -> {
                Pneumatics.getInstance().enableCompressor();
            };

            Runnable runMethod = () -> {};

            Runnable endMethod = () -> {
                Pneumatics.getInstance().disableCompressor();
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
        Robot.autonomousRunner.add(
            PneumaticsActions.defaultPneumaticsAction()
        );

        Robot.teleopRunner.add(
            PneumaticsActions.defaultPneumaticsAction()
        );
    }
}