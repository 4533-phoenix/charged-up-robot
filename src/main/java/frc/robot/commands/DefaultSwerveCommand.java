package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.controls.PSController.Button;
import frc.robot.controls.PSController.Side;
import frc.robot.subsystems.Swerve.DriveSpeed;
import frc.robot.subsystems.*;

public class DefaultSwerveCommand extends CommandBase {
    private final Swerve mSwerve;

    private Supplier<Translation2d> swerveTranslation;
    private Supplier<Double> swerveRotation;

    public DefaultSwerveCommand(Swerve swerve, Supplier<Translation2d> swerveTranslation, Supplier<Double> swerveRotation) {
        mSwerve = swerve;

        this.swerveTranslation = swerveTranslation;
        this.swerveRotation = swerveRotation;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        mSwerve.drive(this.swerveTranslation.get(), this.swerveRotation.get(), mSwerve.getFieldRelative(), true);
    }
}
