// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.controls.PSController;
import frc.robot.subsystems.Auto;
import frc.libs.java.actionLib.ActionRunner;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public final class Robot extends TimedRobot {
  public static final ActionRunner autonomousRunner = new ActionRunner();
  public static final ActionRunner teleopRunner = new ActionRunner();

  public static final PSController driverController = new PSController(OIConstants.DRIVER_CONTROLLER_PORT);
  public static final PSController operatorController = new PSController(OIConstants.OPERATOR_CONTROLLER_PORT);

  public static final SendableChooser<String> chooser = new SendableChooser<String>();

  @Override
  public void robotInit() {
    RobotContainer.queryInitialActions();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    autonomousRunner.add(
      Auto.getInstance().getAutonomous(chooser.getSelected())
    );

    autonomousRunner.enable();
  }

  @Override
  public void autonomousPeriodic() {
    autonomousRunner.run();
  }

  @Override
  public void teleopInit() {
    autonomousRunner.disable();

    teleopRunner.enable();
  }

  @Override
  public void teleopPeriodic() {
    teleopRunner.run();
  }

  @Override
  public void disabledInit() {
    autonomousRunner.disable();

    teleopRunner.disable();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    autonomousRunner.disable();

    teleopRunner.disable();
  }

  @Override
  public void testPeriodic() {}
}
