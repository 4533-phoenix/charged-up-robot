package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.controls.PSController.Button;
import frc.robot.subsystems.*;

public class DefaultSwerveCommand extends CommandBase {
    private final Swerve mSwerve;

    public DefaultSwerveCommand(Swerve swerve) {
        mSwerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if (Robot.driverController.getButton(Button.RB)) {
            mSwerve.slowMode = true;
        } else {
            mSwerve.slowMode = false;
        }
            
        if (Robot.driverController.getButton(Button.LB)) {
            mSwerve.drive(mSwerve.getSwerveTranslation(), mSwerve.getSwerveRotation(), false, true);
        } else {
            mSwerve.drive(mSwerve.getSwerveTranslation(), mSwerve.getSwerveRotation(), true, true);
        }

        if (Robot.driverController.getButton(Button.START)) {
            mSwerve.zeroYaw();
        }
    }
}
