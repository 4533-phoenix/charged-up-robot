package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.WaitForGamepieceCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Extension.ExtensionState;


public class OverChargeStationPickup extends SequentialCommandGroup {
    public OverChargeStationPickup(Swerve swerve, Gripper gripper, Extension extension) {
        PathPlannerTrajectory path1 = PathPlanner.loadPath("Over Charge Station", 3.5, 2.5);

        final Pose2d initialPose;

        if (DriverStation.getAlliance().equals(Alliance.Red)) {
            initialPose = new Pose2d(16.53 - path1.getInitialState().poseMeters.getX(), path1.getInitialState().poseMeters.getY(), path1.getInitialState().poseMeters.getRotation());
        } else {
            initialPose = path1.getInitialState().poseMeters;
        }

        addRequirements(swerve, extension, gripper);    
        addCommands(
            new InstantCommand(() -> swerve.resetPoseEstimator(swerve.getGyroRotation(), swerve.getModulePositions(), initialPose), swerve),
            new InstantCommand(() -> gripper.disableGripper(), gripper),
            new InstantCommand(() -> extension.updateExtensionState(ExtensionState.ABOVE_MATCH_START)),
            new WaitCommand(0.5),
            new InstantCommand(() -> extension.updateExtensionState(ExtensionState.GROUND_HIGH_INTAKE)),
            new ParallelCommandGroup(
                swerve.followTrajectoryCommand(path1),
                new WaitForGamepieceCommand(gripper).withTimeout(path1.getTotalTimeSeconds())),
            new InstantCommand(() -> gripper.enableGripper(), gripper),
            new InstantCommand(() -> extension.updateExtensionState(ExtensionState.OFF_GROUND), extension),
            new InstantCommand(() -> swerve.enableChargeStation(false), swerve),
            new WaitCommand(0.25),
            new InstantCommand(() -> swerve.adjustChargeStation(), swerve)
        );
    }
}
