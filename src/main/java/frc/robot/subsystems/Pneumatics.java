package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.*;

public final class Pneumatics implements Subsystem {
    private static Pneumatics mInstance;

    private final Compressor compressor = new Compressor(GlobalConstants.PCM_ID, PneumaticsModuleType.CTREPCM);

    public Pneumatics() {}

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

    @Override
    public void periodic() {}
}