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
        if (Robot.driverController.getButton(Button.A)) {
            mClimber.raiseClimber();
        } else if (Robot.driverController.getButton(Button.B)) {
            mClimber.lowerClimber();
        }
    }
}
