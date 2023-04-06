package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.controls.PSController.Button;
import frc.robot.subsystems.*;

public class DefaultClimberCommand extends CommandBase {
    private final Climber mClimber;

    public DefaultClimberCommand(Climber climber) {
        mClimber = climber;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        if (Robot.operatorController.getButton(Button.START)) {
            mClimber.releaseClimber();
        }

        if (Robot.operatorController.getButton(Button.BACK)) {
            mClimber.raiseClimber();
        } else {
            mClimber.stopClimber();
        }
    }
}
