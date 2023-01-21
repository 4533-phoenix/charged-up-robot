package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.loops.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Pneumatics extends Subsystem {
    public static Pneumatics mInstance;

    private final Compressor compressor = new Compressor(GlobalConstants.PCM_ID, PneumaticsModuleType.CTREPCM);

    public void enableCompressor() {
        compressor.enableDigital();
    }

    public void disableCompressor() {
        compressor.disable();
    }

    public static Pneumatics getInstance() {
        if (mInstance == null) {
            mInstance = new Pneumatics();
        }
        return mInstance;
    }

    public Pneumatics() {}

    private static final class PneumaticsLoops {
        private Pneumatics mPneumatics = Pneumatics.getInstance();

        public Loop defaultPneumaticsLoop() {
            return new Loop() {
                @Override
                public void onStart(double timestamp) {
                    mPneumatics.enableCompressor();
                }

                @Override
                public void onLoop(double timestamp) {}

                @Override
                public void onStop(double timestamp) {
                    mPneumatics.disableCompressor();
                }
            };
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        PneumaticsLoops pneumaticsLoops = new PneumaticsLoops();

        mEnabledLooper.register(pneumaticsLoops.defaultPneumaticsLoop());
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void writeToDashboard() {}
}