// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.joysticks.IllegalJoystickTypeException;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.robot.subsystems.drive.SwerveDiagnosticDriveTrain;
import frc.robot.subsystems.drive.YagslDriveTrain;
import frc.utils.Alert;
import frc.utils.Alert.AlertType;
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

    private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
    private static final double lowBatteryVoltage = 10.0;
    private final Timer canErrorTimer = new Timer();
    private final Timer canErrorTimerInitial = new Timer();
    private final Alert canErrorAlert =
        new Alert("CAN errors detected, robot may not be controllable.", AlertType.ERROR);
    private final Alert lowBatteryAlert =
        new Alert(
            "Battery voltage is very low, consider turning off the robot or replacing the battery.",
            AlertType.WARNING);

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
            // Start timers
            canErrorTimer.reset();
            canErrorTimer.start();
            canErrorTimerInitial.reset();
            canErrorTimerInitial.start();
        }

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        try {
            m_robotContainer = new RobotContainer();
        } catch (Exception e) {
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

        // Update CAN error alert
        var canStatus = RobotController.getCANStatus();
        if (canStatus.receiveErrorCount > 0 || canStatus.transmitErrorCount > 0) {
            canErrorTimer.reset();
        }
        canErrorAlert.set(
            !canErrorTimer.hasElapsed(canErrorTimeThreshold)
                && canErrorTimerInitial.hasElapsed(canErrorTimeThreshold));

        if (RobotController.getBatteryVoltage() < lowBatteryVoltage) {
            lowBatteryAlert.set(true);
        }
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        m_robotContainer.setAlliance();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
        System.out.println("DisabledExit");
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        // schedule the autonomous command (example)
        System.out.println("AutoInit");
        m_robotContainer.setAlliance();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousExit() {
        System.out.println("AutoExit");
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        System.out.println("TeleopInit");

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        m_robotContainer.setAlliance();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
        System.out.println("TeleopExit");
    }

    @Override
    public void testInit() {
        System.out.println("TestInit");
        m_robotContainer.setAlliance();
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
        System.out.println("TestExit");
    }


    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
        System.out.println("SimulationInit");
        m_robotContainer.setAlliance();
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }

}
