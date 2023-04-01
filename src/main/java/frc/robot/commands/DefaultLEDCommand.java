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
    public void execute() {
      switch(Robot.operatorController.getPOV()) {
        case 0:
            mled.setLEDState(LEDState.YELLOW);
          break;
        case 90:
            mled.setLEDState(LEDState.OFF);
          break;
        case 180:
            mled.setLEDState(LEDState.PURPLE);
          break;
        case 270:
            mled.setLEDState(LEDState.YELLOW_AND_BLUE);
          break;
        default:
          break;
      }
    }
}
