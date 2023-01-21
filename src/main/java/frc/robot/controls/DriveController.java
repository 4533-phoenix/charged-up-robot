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

    // DRIVER CONTROLS
    
    public Translation2d getSwerveTranslation() {
        double forwardAxis = this.getAxis(Side.LEFT, Axis.Y);
        double strafeAxis = this.getAxis(Side.LEFT, Axis.X);

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        double dist = tAxes.getNorm();

        if (Math.abs(tAxes.getNorm()) < swerveDeadband) {
            return new Translation2d();
        } else {
            return new Translation2d(forwardAxis * dist, strafeAxis * dist);
        }
    }

    public double getSwerveRotation() {
        double rotAxis = Math.pow(this.getAxis(Side.RIGHT, Axis.X), 3) * DriveConstants.DRIVE_MAX_VELOCITY;

        if (Math.abs(rotAxis) < swerveDeadband) {
            return 0.0;
        } else {
            return rotAxis;
        }
    }
}