// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.utils.Alert;
import frc.utils.Alert.AlertType;
import frc.utils.LoggerWrapper;
import frc.robot.RobotState.StatePeriod;

import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

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
    private RobotState m_state;
    private int m_iteration = 0;

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

        // Push the trajectory to Field2d.

        System.out.println("[Init] Robot");

        m_state = RobotState.getInstance();

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
            AutoLogOutputManager.addPackage("frc.robot");
            AutoLogOutputManager.addPackage("frc.utils");

            if (isReal()) {

                if (LogfileChecker(Constants.logFolder0)) {
                    LoggerWrapper.addDataReceiver(new WPILOGWriter(Constants.logFolder0)); // Log to a USB stick ("/U/logs")
                } else if (LogfileChecker((Constants.logFolder1))) {
                    LoggerWrapper.addDataReceiver(new WPILOGWriter(Constants.logFolder1)); // Log to a USB stick ("/U/logs")
                } else if (LogfileChecker((Constants.logFolder2))) {
                    LoggerWrapper.addDataReceiver(new WPILOGWriter(Constants.logFolder2)); // Log to a USB stick ("/U/logs")
                } else if (LogfileChecker((Constants.logFolder3))) {
                    LoggerWrapper.addDataReceiver(new WPILOGWriter(Constants.logFolder3)); // Log to a USB stick ("/U/logs")
                } else if (LogfileChecker((Constants.logFolder4))) {
                    LoggerWrapper.addDataReceiver(new WPILOGWriter(Constants.logFolder4)); // Log to a USB stick ("/U/logs")
                } else if (LogfileChecker((Constants.logFolder5))) {
                    LoggerWrapper.addDataReceiver(new WPILOGWriter(Constants.logFolder5)); // Log to a USB stick ("/U/logs")
                } else if (LogfileChecker((Constants.logFolder6))) {
                    LoggerWrapper.addDataReceiver(new WPILOGWriter(Constants.logFolder6)); // Log to a USB stick ("/U/logs")
                } else if (LogfileChecker((Constants.logFolder7))) {
                    LoggerWrapper.addDataReceiver(new WPILOGWriter(Constants.logFolder7)); // Log to a USB stick ("/U/logs")
                } else if (LogfileChecker((Constants.logFolder8))) {
                    LoggerWrapper.addDataReceiver(new WPILOGWriter(Constants.logFolder8)); // Log to a USB stick ("/U/logs")
                } else if (LogfileChecker((Constants.logFolder9))) {
                    LoggerWrapper.addDataReceiver(new WPILOGWriter(Constants.logFolder9)); // Log to a USB stick ("/U/logs")
                } else {
                    System.out.println("No Log file Chosen!");
                }

                LoggerWrapper.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
                LoggerWrapper.enablePowerDistributionLogging();
            } else if (!isSimulation()) {
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
                LoggerWrapper.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                LoggerWrapper.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
            }
            logActiveCommand();
            LoggerWrapper.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
        }

        try {
            m_robotContainer = new RobotContainer();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        System.out.println("[DONE] Robot");

    }

    private boolean LogfileChecker(String file) {
        File testFile = new File(file+"/hello.txt");
        System.out.print("Checking log location " + testFile.getAbsolutePath());
        boolean check;
        try {
            testFile.createNewFile();
            check = true;
            testFile.delete();
        } catch (Exception e) {
            System.out.print(", exception = " + e.getMessage());
            check = false;
        }
        System.out.println(", check = " + check);

        return check;
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

    private void logActiveCommand() {
        // Log active commands
        Map<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction =
            (Command command, Boolean active) -> {
                String name = command.getName();
                int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
                commandCounts.put(name, count);
                LoggerWrapper.recordOutput(
                    "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
                LoggerWrapper.recordOutput("CommandsAll/" + name, count > 0);
            };
        CommandScheduler.getInstance()
            .onCommandInitialize(
                (Command command) -> {
                    logCommandFunction.accept(command, true);
                });
        CommandScheduler.getInstance()
            .onCommandFinish(
                (Command command) -> {
                    logCommandFunction.accept(command, false);
                });
        CommandScheduler.getInstance()
            .onCommandInterrupt(
                (Command command) -> {
                    logCommandFunction.accept(command, false);
                });
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        System.out.println("DisabledInit");
        m_state.setPeriod(StatePeriod.DISABLED);
        m_robotContainer.updateAlliance();
    }

    @Override
    public void disabledPeriodic() {
        if (++m_iteration % 25 == 0) {
            m_robotContainer.updateAlliance();
        }
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
        m_state.setPeriod(StatePeriod.AUTONOMOUS);
        m_robotContainer.updateAlliance();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            System.out.println("Auto Command set");
            m_autonomousCommand.schedule();
        } else {
            System.out.println("Auto Command NULL");
        }

        System.out.println("Auto Init Pos X: " + RobotState.getInstance().getPose().getX());
        System.out.println("Auto Init Pos Y: " + RobotState.getInstance().getPose().getY());
        System.out.println("Auto Init Pos rot: " + RobotState.getInstance().getPose().getRotation().getDegrees());
    }

    @Override
    public void autonomousExit() {
        System.out.println("AutoExit");
        System.out.println("Auto exti Pos X: " + RobotState.getInstance().getPose().getX());
        System.out.println("Auto exit Pos Y: " + RobotState.getInstance().getPose().getY());
        System.out.println("Auto exit Pos rot: " + RobotState.getInstance().getPose().getRotation().getDegrees());    }

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

        m_robotContainer.updateAlliance();
        m_state.setPeriod(StatePeriod.TELEOP);
        if (!m_state.getDidAuto()) { // If we did auto, leave the robot where it is.
            m_robotContainer.resetInitialPose();
        }

        if (Constants.Toggles.useClimber && Constants.Climber.autoHome) {
            m_robotContainer.autoHome();
        }

        System.out.println("tele Init Pos X: " + RobotState.getInstance().getPose().getX());
        System.out.println("tele Init Pos Y: " + RobotState.getInstance().getPose().getY());
        System.out.println("tele Init Pos rot: " + RobotState.getInstance().getPose().getRotation().getDegrees());
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
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        m_robotContainer.updateAlliance();
        m_state.setPeriod(StatePeriod.TEST);
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
        m_robotContainer.updateAlliance();
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }

}
