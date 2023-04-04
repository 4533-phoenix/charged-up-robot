package frc.robot;

import java.util.Map;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultClimberCommand;
import frc.robot.commands.DefaultExtensionCommand;
import frc.robot.commands.DefaultGripperCommand;
import frc.robot.commands.DefaultLEDCommand;
import frc.robot.commands.DefaultSwerveCommand;
import frc.robot.commands.ForkliftClimbCommand;
import frc.robot.commands.SetClimbPositionCommand;
import frc.robot.commands.autos.ChargeStationScoreAndEnable;
import frc.robot.commands.autos.LaneTwoPiece;
import frc.robot.commands.autos.PathPlannerTest;
import frc.robot.commands.autos.RightLeftScoreAndLeave;
import frc.robot.controls.PSController.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    private final Extension extension = new Extension();
    private final Gripper gripper = new Gripper();
    private final LED led = new LED();
    private final Pneumatics pneumatics = new Pneumatics();
    private final Swerve swerve = new Swerve();
    private final Climber climber = new Climber();

    public RobotContainer() {
        CommandScheduler.getInstance().registerSubsystem(extension);
        CommandScheduler.getInstance().registerSubsystem(gripper);
        CommandScheduler.getInstance().registerSubsystem(led);
        CommandScheduler.getInstance().registerSubsystem(pneumatics);
        CommandScheduler.getInstance().registerSubsystem(swerve);
        CommandScheduler.getInstance().registerSubsystem(climber);

        led.setDefaultCommand(new DefaultLEDCommand(led));
        swerve.setDefaultCommand(new DefaultSwerveCommand(swerve));
        gripper.setDefaultCommand(new DefaultGripperCommand(gripper));
        extension.setDefaultCommand(new DefaultExtensionCommand(extension));
        climber.setDefaultCommand(new DefaultClimberCommand(climber));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // new Trigger(() -> Robot.operatorController.getButton(Button.BACK)).onTrue(new SetClimbPositionCommand(this.extension, this.climber));
        // new Trigger(() -> Robot.operatorController.getButton(Button.START)).onTrue(new ForkliftClimbCommand(this.extension, this.climber));
    }

    public final Map<String, Command> autoCommands = Map.ofEntries(
        Map.entry("PathPlanner Test", new PathPlannerTest(this.swerve, this.extension, this.gripper)),
        Map.entry("Right/Left Score and Leave", new RightLeftScoreAndLeave(this.swerve, this.extension, this.gripper)),
        Map.entry("Charge Station Score and Enable", new ChargeStationScoreAndEnable(this.swerve, this.extension, this.gripper)),
        Map.entry("Lane Two Piece", new LaneTwoPiece(this.swerve, this.gripper, this.extension))
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

    public Climber getClimber() {
        return this.climber;
    }
}
