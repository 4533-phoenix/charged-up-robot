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
        mSwerve.setSlowMode(Robot.driverController.getButton(Button.RB));
        mSwerve.drive(mSwerve.getSwerveTranslation(), mSwerve.getSwerveRotation(), !Robot.driverController.getButton(Button.LB), true);

        if (Robot.driverController.getButton(Button.START)) {
            mSwerve.zeroYaw();
        }
    }
}
