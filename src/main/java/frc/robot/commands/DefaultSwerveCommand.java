package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.PSController.Button;
import frc.robot.controls.PSController.Side;
import frc.robot.subsystems.Swerve.DriveSpeed;
import frc.robot.subsystems.*;

public class DefaultSwerveCommand extends Command {
    private final Swerve mSwerve;

    public DefaultSwerveCommand(Swerve swerve) {
        mSwerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if (Robot.driverController.getButton(Button.RB)) {
            mSwerve.setDriveSpeed(DriveSpeed.SLOW);
        } else if (Robot.driverController.getTrigger(Side.RIGHT)) {
            mSwerve.setDriveSpeed(DriveSpeed.FAST);
        } else {
            mSwerve.setDriveSpeed(DriveSpeed.STANDARD);
        }
        
        mSwerve.drive(mSwerve.getSwerveTranslation(), mSwerve.getSwerveRotation(), !Robot.driverController.getButton(Button.LB), true);

        if (Robot.driverController.getButton(Button.START)) {
            mSwerve.zeroYaw();
        }
    }
}
