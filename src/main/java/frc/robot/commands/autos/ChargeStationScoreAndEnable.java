package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Extension.ExtensionState;

public class ChargeStationScoreAndEnable extends SequentialCommandGroup {
    public ChargeStationScoreAndEnable(Swerve swerve, Extension extension, Gripper gripper) {
        addRequirements(swerve, extension, gripper);
        addCommands(
            new InstantCommand(() -> extension.updateExtensionState(ExtensionState.HIGH_ROW), extension),
            new WaitCommand(1.25),
            new InstantCommand(() -> gripper.disableGripper(), gripper),
            new WaitCommand(0.5),
            new InstantCommand(() -> swerve.straightenWheels(1.0), swerve),
            new WaitCommand(1.0),
            new InstantCommand(() -> extension.updateExtensionState(ExtensionState.OFF_GROUND)), 
            new InstantCommand(() -> swerve.enableChargeStation(false), swerve),
            new WaitCommand(0.25),
            new InstantCommand(() -> swerve.adjustChargeStation(), swerve)
        );
    }
}