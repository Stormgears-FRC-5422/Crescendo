// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Drive;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Toggles;
import frc.robot.commands.JoyStickDrive;
import frc.robot.joysticks.CrescendoJoystick;
import frc.robot.joysticks.CrescendoJoystickFactory;
import frc.robot.joysticks.IllegalJoystickTypeException;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;
import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.utils.joysticks.StormLogitechController;
import frc.utils.joysticks.StormXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);

    NavX navX;

    DrivetrainBase drivetrainBase;

    StormXboxController xboxController;

    StormLogitechController logitechController;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() throws IllegalDriveTypeException, IllegalJoystickTypeException {
        System.out.println("[Init] RobotContainer");

        CrescendoJoystick joystick = null;
        if (Toggles.useDrive) {
            System.out.println("Create drive type " + Drive.driveType);
            drivetrainBase = DrivetrainFactory.getInstance(Drive.driveType);
            joystick = CrescendoJoystickFactory.getInstance(Drive.joystick);
        }

        JoyStickDrive driveWithJoystick = new JoyStickDrive(
                drivetrainBase, joystick);
        drivetrainBase.setDefaultCommand(driveWithJoystick);


        if (Toggles.useNavX) {
            navX = new NavX();
        }

        // Configure the trigger bindings
        configureBindings();

        System.out.println("[DONE] RobotContainer");
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        System.out.println("[Init] configureBindings");

        System.out.println("[DONE] configureBindings");
    }

}
