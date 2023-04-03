package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.WaitForGamepieceCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Extension.ExtensionState;


public class LaneTwoPiece extends SequentialCommandGroup {
    public LaneTwoPiece(Swerve swerve, Gripper gripper, Extension extension) {
        PathPlannerTrajectory path1 = PathPlanner.loadPath("Lane Pickup", 2.0, 3.0);
        PathPlannerTrajectory path2 = PathPlanner.loadPath("Lane Score", 2.0, 3.0);

        addRequirements(swerve, extension, gripper);
        addCommands(
            new ParallelCommandGroup(
                swerve.followTrajectoryCommand(path1),
                new WaitForGamepieceCommand(gripper).withTimeout(path1.getTotalTimeSeconds())),
            new InstantCommand(() -> extension.updateExtensionState(ExtensionState.OFF_GROUND), extension),
            swerve.followTrajectoryCommand(path2),
            new InstantCommand(() -> extension.updateExtensionState(ExtensionState.MIDDLE_ROW), extension),
            new WaitCommand(0.75),
            new InstantCommand(() -> gripper.disableGripper(), gripper)
        );
    }
}
