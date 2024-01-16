// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.utils.LoggerWrapper;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import static frc.robot.Constants.Toggles.useAdvantageKit;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Record metadata
    System.out.println("[Init] Robot");

    if (useAdvantageKit) {
      System.out.println("[Init] Starting AdvantageKit");
    }
    LoggerWrapper.recordMetadata("Robot", Constants.robotName);
    LoggerWrapper.recordMetadata("RuntimeType", getRuntimeType().toString());
    LoggerWrapper.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    LoggerWrapper.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    LoggerWrapper.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    LoggerWrapper.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    LoggerWrapper.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0 -> LoggerWrapper.recordMetadata("GitDirty", "All changes committed");
      case 1 -> LoggerWrapper.recordMetadata("GitDirty", "Uncomitted changes");
      default -> LoggerWrapper.recordMetadata("GitDirty", "Unknown");
    }

    if (isReal()) {
      LoggerWrapper.addDataReceiver(new WPILOGWriter(Constants.logFolder)); // Log to a USB stick ("/U/logs")
      LoggerWrapper.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      LoggerWrapper.enablePowerDistributionLogging();
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      LoggerWrapper.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      LoggerWrapper.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }
    LoggerWrapper.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    try {
      m_robotContainer = new RobotContainer();
    } catch (IllegalDriveTypeException e) {
      throw new RuntimeException(e);
    }
    System.out.println("[DONE] Robot");
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit() {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic() {
  }
}
