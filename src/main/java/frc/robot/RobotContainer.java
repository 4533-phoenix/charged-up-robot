package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
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
import frc.robot.subsystems.Extension.ExtensionState;
import frc.robot.subsystems.LED.LEDState;
import frc.robot.subsystems.Swerve.DriveSpeed;

public class RobotContainer {
    private final Extension extension = new Extension();
    private final Gripper gripper = new Gripper();
    private final LED led = new LED();
    private final Pneumatics pneumatics = new Pneumatics();
    private final Swerve swerve = new Swerve();

    public static final CommandXboxController driverController = new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    public static final CommandXboxController operatorController = new CommandXboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

    public RobotContainer() {
        CommandScheduler.getInstance().registerSubsystem(extension);
        CommandScheduler.getInstance().registerSubsystem(gripper);
        CommandScheduler.getInstance().registerSubsystem(led);
        CommandScheduler.getInstance().registerSubsystem(pneumatics);
        CommandScheduler.getInstance().registerSubsystem(swerve);

        led.setDefaultCommand(new DefaultLEDCommand(led));
        swerve.setDefaultCommand(new DefaultSwerveCommand(swerve,
            this::getSwerveTranslation,
            this::getSwerveRotation));
        gripper.setDefaultCommand(new DefaultGripperCommand(gripper));
        extension.setDefaultCommand(new DefaultExtensionCommand(extension));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        operatorController.y().onTrue(new InstantCommand(() -> extension.updateExtensionState(ExtensionState.GROUND_LOW_INTAKE), extension));
        operatorController.x().onTrue(new InstantCommand(() -> extension.updateExtensionState(ExtensionState.OFF_GROUND), extension));
        operatorController.b().onTrue(new InstantCommand(() -> extension.updateExtensionState(ExtensionState.MIDDLE_ROW), extension));
        operatorController.a().onTrue(new InstantCommand(() -> extension.updateExtensionState(ExtensionState.OFF_GROUND), extension));
        operatorController.rightBumper().onTrue(new InstantCommand(() -> extension.updateExtensionState(ExtensionState.HIGHER), extension));
        operatorController.leftBumper().onTrue(new InstantCommand(() -> extension.updateExtensionState(ExtensionState.LOWER), extension));

        operatorController.rightTrigger().onTrue(new InstantCommand(() -> gripper.enableGripper(), gripper));
        driverController.leftTrigger().onTrue(new InstantCommand(() -> gripper.disableGripper(), gripper));
        operatorController.leftTrigger().onTrue(new InstantCommand(() -> gripper.disableGripper(), gripper));

        operatorController.povUp().onTrue(new InstantCommand(() -> led.setLEDState(LEDState.YELLOW), led));
        operatorController.povRight().onTrue(new InstantCommand(() -> led.setLEDState(LEDState.OFF), led));
        operatorController.povDown().onTrue(new InstantCommand(() -> led.setLEDState(LEDState.PURPLE), led));
        operatorController.povLeft().onTrue(new InstantCommand(() -> led.setLEDState(LEDState.YELLOW_AND_BLUE), led));

        Trigger standardDriveTrigger = driverController.rightBumper().negate().and(driverController.rightTrigger().negate());
        
        driverController.rightBumper().onTrue(new InstantCommand(() -> swerve.setDriveSpeed(DriveSpeed.SLOW), swerve));
        driverController.rightTrigger().onTrue(new InstantCommand(() -> swerve.setDriveSpeed(DriveSpeed.FAST), swerve));
        standardDriveTrigger.onTrue(new InstantCommand(() -> swerve.setDriveSpeed(DriveSpeed.FAST), swerve));

        driverController.leftBumper().onTrue(new InstantCommand(() -> swerve.setFieldRelative(false), swerve));
        driverController.leftBumper().onFalse(new InstantCommand(() -> swerve.setFieldRelative(true), swerve));

        driverController.start().onTrue(new InstantCommand(() -> swerve.zeroYaw(), swerve));
    }

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

    public Translation2d getSwerveTranslation() {
        double forwardAxis = driverController.getLeftY();
        double strafeAxis = -driverController.getLeftX();

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        double dist = tAxes.getNorm();

        if (Math.abs(tAxes.getNorm()) < OIConstants.DRIVE_DEADBAND) {
            return new Translation2d();
        } else {
            return new Translation2d(forwardAxis * dist, strafeAxis * dist);
        }
    }

    public double getSwerveRotation() {
        double rotAxis = -Math.pow(driverController.getRightX(), 3);

        if (Math.abs(rotAxis) < OIConstants.DRIVE_DEADBAND) {
            return 0.0;
        } else {
            return rotAxis;
        }
    }
}
