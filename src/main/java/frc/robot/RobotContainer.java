package frc.robot;

import java.util.Map;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.DefaultClimberCommand;
import frc.robot.commands.DefaultExtensionCommand;
import frc.robot.commands.DefaultGripperCommand;
import frc.robot.commands.DefaultLEDCommand;
import frc.robot.commands.DefaultSwerveCommand;
import frc.robot.commands.autos.BumpThreePiece;
import frc.robot.commands.autos.BumpTwoPiece;
import frc.robot.commands.autos.ChargeStationScoreAndEnable;
import frc.robot.commands.autos.LaneThreePiece;
import frc.robot.commands.autos.LaneTwoPiece;
import frc.robot.commands.autos.OverChargeStationPickup;
import frc.robot.commands.autos.PathPlannerTest;
import frc.robot.commands.autos.RightLeftScoreAndLeave;
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

    private void configureButtonBindings() {}

    public final Map<String, Command> autoCommands = Map.ofEntries(
        Map.entry("PathPlanner Test", new PathPlannerTest(this.swerve, this.extension, this.gripper)),
        Map.entry("Right/Left Score and Leave", new RightLeftScoreAndLeave(this.swerve, this.extension, this.gripper)),
        Map.entry("Charge Station Score and Enable", new ChargeStationScoreAndEnable(this.swerve, this.extension, this.gripper)),
        Map.entry("Lane Two Piece", new LaneTwoPiece(this.swerve, this.gripper, this.extension)),
        Map.entry("Lane Three Piece", new LaneThreePiece(this.swerve, this.gripper, this.extension)),
        Map.entry("Bump Two Piece", new BumpTwoPiece(this.swerve, this.gripper, this.extension)),
        Map.entry("Bump Three Piece", new BumpThreePiece(this.swerve, this.gripper, this.extension)),
        Map.entry("Over Charge Station Pickup", new OverChargeStationPickup(this.swerve, this.gripper, this.extension))
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
