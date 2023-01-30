package frc.robot.controls;

import frc.robot.Constants.*;
import edu.wpi.first.math.geometry.Translation2d;

public final class DriveController extends PSController {
    private final double swerveDeadband = OIConstants.DRIVE_DEADBAND;

    private static DriveController mInstance = null;

    public static DriveController getInstance() {
        if (mInstance == null) {
            mInstance = new DriveController();
        }

        return mInstance;
    }

    private DriveController() {
        super(OIConstants.DRIVER_CONTROLLER_PORT);
    }
}