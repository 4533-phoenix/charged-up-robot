package frc.robot;

//import java.util.Map;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.DefaultSwerveCommand;
import frc.robot.subsystems.*;

public class RobotContainer {
    private final Swerve swerve = new Swerve();

    public RobotContainer() {
        CommandScheduler.getInstance().registerSubsystem(swerve);

        swerve.setDefaultCommand(new DefaultSwerveCommand(swerve));

        configureButtonBindings();
    }

    private void configureButtonBindings() {}




    public Swerve getSwerve() {
        return this.swerve;
    }
}
