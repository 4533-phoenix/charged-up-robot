package frc.robot;

import java.util.Map;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.DefaultExtensionCommand;
import frc.robot.commands.DefaultGripperCommand;
import frc.robot.commands.DefaultSwerveCommand;
import frc.robot.subsystems.*;

public class RobotContainer {
    private final Extension extension = new Extension();
    private final Gripper gripper = new Gripper();
    private final LED led = new LED();
    private final Pneumatics pneumatics = new Pneumatics();
    private final Swerve swerve = new Swerve();

    public RobotContainer() {
        CommandScheduler.getInstance().registerSubsystem(extension);
        CommandScheduler.getInstance().registerSubsystem(gripper);
        CommandScheduler.getInstance().registerSubsystem(led);
        CommandScheduler.getInstance().registerSubsystem(pneumatics);
        CommandScheduler.getInstance().registerSubsystem(swerve);

        swerve.setDefaultCommand(new DefaultSwerveCommand(swerve));
        gripper.setDefaultCommand(new DefaultGripperCommand(gripper));
        extension.setDefaultCommand(new DefaultExtensionCommand(extension));

        configureButtonBindings();
    }

    public void configureButtonBindings() {

    }

    private static final Map<String, Command> autoCommands = Map.ofEntries(
        Map.entry("PathPlanner Test", null),
        Map.entry("Right/Left Score and Leave", null),
        Map.entry("Charge Station Score and Enable", null),
        Map.entry("Do Nothing", null)
    );

    public Extension getExtension() {
        return this.extension;
    }

    public Gripper getGripper() {
        return this.gripper;
    }

    public LED getLED() {
        return this.led;
    }

    public Pneumatics getPneumatics() {
        return this.pneumatics;
    }

    public Swerve getSwerve() {
        return this.swerve;
    }
}
