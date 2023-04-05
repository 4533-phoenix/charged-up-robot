package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class PathPlannerTest extends SequentialCommandGroup {
    public PathPlannerTest(Swerve swerve, Extension extension, Gripper gripper) {
        PathPlannerTrajectory path = PathPlanner.loadPath("Over Charge Station", 2.0, 3.0);

        addRequirements(swerve, extension, gripper);
        addCommands(
            swerve.followTrajectoryCommand(path)
        );
    }
}
