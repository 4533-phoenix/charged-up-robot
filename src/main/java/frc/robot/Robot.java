// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public final class Robot extends TimedRobot {
  public static final RobotContainer robotContainer = new RobotContainer();

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
  public void teleopPeriodic() {}

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
