package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LEDState;

public class DefaultLEDCommand extends CommandBase {
    private final LED mled;
  
    public DefaultLEDCommand(LED led) {
      mled = led;

      addRequirements(mled);
    }
    
    @Override
    public void execute() {}
}
