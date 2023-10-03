// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.controls.PSController;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public final class Robot extends TimedRobot {
  public static final RobotContainer robotContainer = new RobotContainer();

  public static final PSController driverController = new PSController(OIConstants.DRIVER_CONTROLLER_PORT);
  public static final PSController operatorController = new PSController(OIConstants.OPERATOR_CONTROLLER_PORT);

  public static final SendableChooser<String> chooser = new SendableChooser<String>();

  private String autoSelected = chooser.getSelected();
  private Command autoCommand;

  @Override
  public void robotInit() {
    chooser.setDefaultOption("PathPlanner Test", "PathPlanner Test");
    chooser.addOption("Right/Left Score and Leave", "Right/Left Score and Leave");
    chooser.addOption("Charge Station Score and Enable", "Charge Station Score and Enable");
    chooser.addOption("Lane Two Piece", "Lane Two Piece");
    chooser.addOption("Lane Three Piece", "Lane Three Piece");
    chooser.addOption("Bump Two Piece", "Bump Two Piece");
    chooser.addOption("Bump Three Piece", "Bump Three Piece");
    chooser.addOption("Over Charge Station Pickup", "Over Charge Station Pickup");
    SmartDashboard.putData("Select Auto", chooser);

    SmartDashboard.putBoolean("Drive PID Test Is Activated", false);

    SmartDashboard.putNumber("Steer Angle", 0.0);

    SmartDashboard.putNumber("Drive kP", ModuleConstants.DRIVE_KP);
    SmartDashboard.putNumber("Drive kI", ModuleConstants.DRIVE_KI);
    SmartDashboard.putNumber("Drive kD", ModuleConstants.DRIVE_KI);

    SmartDashboard.putNumber("Steer kP", ModuleConstants.STEER_KP);
    SmartDashboard.putNumber("Steer kI", ModuleConstants.STEER_KI);
    SmartDashboard.putNumber("Steer kD", ModuleConstants.STEER_KD);
    
    UsbCamera gripperCamera = CameraServer.startAutomaticCapture();
    gripperCamera.setResolution(640, 480);

    robotContainer.getPneumatics().enableCompressor();

    robotContainer.getSwerve().zeroGyro();

    robotContainer.getExtension().elbowAbsoluteEncoder.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0);
    robotContainer.getExtension().elbowAbsoluteEncoder.setPositionOffset(ExtensionConstants.ELBOW_ABSOLUTE_ENCODER_OFFSET);
    robotContainer.getExtension().initialAbsoluteEncoderPosition = 1.0 + robotContainer.getExtension().getAbsoluteEncoderAbsolutePosition() - robotContainer.getExtension().elbowAbsoluteEncoder.getPositionOffset();
    robotContainer.getExtension().elbowRelativeEncoder.reset();

    robotContainer.getLED().configureLEDs();

    robotContainer.getGripper().enableGripper();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    autoSelected = chooser.getSelected();
    autoCommand = robotContainer.autoCommands.get(autoSelected);

    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    robotContainer.getExtension().updateExtensionState();
    robotContainer.getExtension().updateElbowController();
  }

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // PID tuning test code

    // Sets the drive motor velocity to 5.0 m/s if drive PID testing is activated on the Smart Dashboard
    double driveMotorVelocity = SmartDashboard.getBoolean("Drive PID Test Is Activated", false) ? 5.0 : 0.0;

    // Sets the steer motor angle to the specified angle from the Smart Dashboard
    Rotation2d steerMotorAngle = Rotation2d.fromDegrees(SmartDashboard.getNumber("Steer Angle", 0.0)); 

    // Sets the drive PID and steer PID values to the specified values from the Smart Dashboard
    for (SwerveModule swerveMod : robotContainer.getSwerve().getSwerveModules()) {
      swerveMod.getDrivePIDController().setPID(
        SmartDashboard.getNumber("Drive kP", ModuleConstants.DRIVE_KP), 
        SmartDashboard.getNumber("Drive kI", ModuleConstants.DRIVE_KI), 
        SmartDashboard.getNumber("Drive kD", ModuleConstants.DRIVE_KD)
      );

      swerveMod.getSteerPIDController().setPID(
        SmartDashboard.getNumber("Steer kP", ModuleConstants.STEER_KP),
        SmartDashboard.getNumber("Steer kI", ModuleConstants.STEER_KI),
        SmartDashboard.getNumber("Steer kD", ModuleConstants.STEER_KD)
      );
    }

    // Sets the swerve modules to drive if activated and to the specified angle
    robotContainer.getSwerve().setModuleStates(
      new SwerveModuleState[]{
        new SwerveModuleState(driveMotorVelocity, steerMotorAngle),
        new SwerveModuleState(driveMotorVelocity, steerMotorAngle),
        new SwerveModuleState(driveMotorVelocity, steerMotorAngle),
        new SwerveModuleState(driveMotorVelocity, steerMotorAngle)
      }
    );
  }

  @Override
  public void disabledInit() {
    robotContainer.getPneumatics().disableCompressor();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
