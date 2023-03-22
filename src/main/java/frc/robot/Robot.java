// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.PathPlannerAutos;
import frc.robot.autos.PathPlannerAutos.WhichAuto;
import frc.robot.subsystems.Camera;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;
  private final SendableChooser<PathPlannerAutos.WhichAuto> m_chooser = new SendableChooser<>();

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    
    m_chooser.setDefaultOption("Test Auto", WhichAuto.testAuto);
    m_chooser.addOption("Charge Center Auto", WhichAuto.charge);
    m_chooser.addOption("Preload", WhichAuto.preload);
    m_chooser.addOption("Preload Mobility Cable", WhichAuto.preloadMobilityCable);
    m_chooser.addOption("Preload Mobility Open", WhichAuto.preloadMobilityOpen);

    SmartDashboard.putData(m_chooser);

    // Sets up the Camera to receive Frames from the USB Webcam and to
    // pass them along to the Driver Station.
    Camera.getInstance().cameraSetup();

    m_robotContainer = new RobotContainer();
   // CommandScheduler.getInstance().schedule(m_robotContainer.goToDriverPosFromBottom);
    SmartDashboard.putString("StatusNormal", "Robot Init");
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = PathPlannerAutos.scorePreloadCube;

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    ctreConfigs = new CTREConfigs();
    SmartDashboard.putString("StatusTest", "Test Init");
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // Cancels all running commands at the start of test mode.
    //CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().enable();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();
    //if (operator.) 
  }
}
