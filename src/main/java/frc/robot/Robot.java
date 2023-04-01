// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LED.LEDState; 
import frc.robot.controls.PSController;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Extension.ExtensionState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public final class Robot extends TimedRobot {
  private final RobotContainer robotContainer = new RobotContainer();

  public static final PSController driverController = new PSController(OIConstants.DRIVER_CONTROLLER_PORT);
  public static final PSController operatorController = new PSController(OIConstants.OPERATOR_CONTROLLER_PORT);

  public static final SendableChooser<String> chooser = new SendableChooser<String>();

  private String autoSelected = chooser.getSelected();

  @Override
  public void robotInit() {
    chooser.setDefaultOption("PathPlanner Test", "PathPlanner Test");
    chooser.addOption("Right/Left Score and Leave", "Right/Left Score and Leave");
    chooser.addOption("Do Nothing", "Do Nothing");
    chooser.addOption("Charge Station Score and Enable", "Charge Station Score and Enable");
    SmartDashboard.putData("Select Auto", chooser);
    
    UsbCamera gripperCamera = CameraServer.startAutomaticCapture();
    gripperCamera.setResolution(640, 480);

    robotContainer.getPneumatics().enableCompressor();

    robotContainer.getSwerve().swervePoseEstimator.resetPosition(
      robotContainer.getSwerve().getGyroRotation(), 
      robotContainer.getSwerve().getModulePositions(), 
      robotContainer.getSwerve().initialPose
    );

    robotContainer.getSwerve().zeroGyro();

    robotContainer.getExtension().elbowAbsoluteEncoder.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0);
    robotContainer.getExtension().elbowAbsoluteEncoder.setPositionOffset(ExtensionConstants.ELBOW_ABSOLUTE_ENCODER_OFFSET);
    robotContainer.getExtension().initialAbsoluteEncoderPosition = 1.0 + robotContainer.getExtension().getAbsoluteEncoderAbsolutePosition() - robotContainer.getExtension().elbowAbsoluteEncoder.getPositionOffset();
    robotContainer.getExtension().elbowRelativeEncoder.reset();

    robotContainer.getLED().configureLEDs();

    robotContainer.getGripper().enableGripper();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    this.autoSelected = chooser.getSelected();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    if (Robot.operatorController.getPOV() == 0) {
      robotContainer.getLED().setLEDState(LEDState.YELLOW);
    } else if (Robot.operatorController.getPOV() == 90) {
      robotContainer.getLED().setLEDState(LEDState.OFF);
    } else if (Robot.operatorController.getPOV() == 180) {
      robotContainer.getLED().setLEDState(LEDState.PURPLE);
    } else if (Robot.operatorController.getPOV() == 270) {
      robotContainer.getLED().setLEDState(LEDState.YELLOW_AND_BLUE);
    }
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
