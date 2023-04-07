package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Extension.ExtensionState;

public class RightLeftScoreAndLeave extends SequentialCommandGroup {
    public RightLeftScoreAndLeave(Swerve swerve, Extension extension, Gripper gripper) {
        addRequirements(swerve, extension, gripper);
        addCommands(
            new InstantCommand(() -> gripper.disableGripper(), gripper),
            new WaitCommand(0.5),
            new InstantCommand(() -> swerve.straightenWheels(1.0), swerve),
            new InstantCommand(() -> swerve.driveDistance(5.0), swerve),
            new InstantCommand(() -> swerve.stopDrive(), swerve)
        );
    }
}