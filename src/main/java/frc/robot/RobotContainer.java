// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Drive;
import frc.robot.Constants.ButtonBoard;
import frc.robot.Constants.Toggles;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JoyStickDrive;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.auto.AutoCommands;
import frc.robot.joysticks.CrescendoJoystick;
import frc.robot.joysticks.CrescendoJoystickFactory;
import frc.robot.joysticks.IllegalJoystickTypeException;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;
import frc.robot.subsystems.drive.IllegalDriveTypeException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


    // **********
    // SubSystems
    private ShooterSubsystem shooter;
    private ShooterCommand shooterCommand;

    private IntakeSubSystem intake;
    private IntakeCommand intakeCommand;


    // **********
    DrivetrainBase drivetrain;
    NavX navX;

    // **********
    // Commands
    // **********
    AutoCommands autoCommands;

    private final LoggedDashboardChooser<Command> autoChooser;


    // **********
    // Fields
    // **********
    final RobotState robotState;

    CrescendoJoystick joystick;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() throws IllegalDriveTypeException, IllegalJoystickTypeException {
        System.out.println("[Init] RobotContainer");
        robotState = RobotState.getInstance();
        setAlliance();

        if (Toggles.useDrive) {
            System.out.println("Create drive type " + Drive.driveType);
            drivetrain = DrivetrainFactory.getInstance(Drive.driveType);
            if (Toggles.useController) {
                joystick = CrescendoJoystickFactory.getInstance(ButtonBoard.driveJoystick, ButtonBoard.driveJoystickPort);
                JoyStickDrive driveWithJoystick = new JoyStickDrive(drivetrain, joystick);
                drivetrain.setDefaultCommand(driveWithJoystick);

            }

            // TODO - for now.  We have to start somewhere.
            Pose2d initialPose = new Pose2d(ButtonBoard.initPoseX, ButtonBoard.initPoseY,
                Rotation2d.fromDegrees(ButtonBoard.initPoseDegrees));

            initialPose = CrescendoField.remapPose(initialPose, robotState.isAllianceBlue());
            drivetrain.resetOdometry(initialPose);
        }

        if (Toggles.useShooter) {
            shooter = new ShooterSubsystem();
            shooterCommand = new ShooterCommand(shooter, joystick);
            shooter.setDefaultCommand(shooterCommand);
        }

        if (Toggles.useIntake) {
            intake = new IntakeSubSystem();
            intakeCommand = new IntakeCommand(intake, joystick);
            intake.setDefaultCommand(intakeCommand);
        }

        if (Toggles.useNavX && !Drive.driveType.equals("YagslDrive")) {
            System.out.println("Create NavX");
            navX = new NavX();
        }

        // Configure the trigger bindings
        configureBindings();



        System.out.println("[DONE] RobotContainer");

//      TODO need to add place for this
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");

        autoCommands = new AutoCommands(drivetrain);
//         need a better way of doing this-don't want to have to write this out everytime
        autoChooser.addOption("test_2m", autoCommands.test2m());
        autoChooser.addOption("4noteAmp", autoCommands.fourNoteAmp());
        autoChooser.addDefaultOption("test_2m", autoCommands.test2m());


    }

    public void setAlliance() {
        // TODO We don't always get a clear alliance from the driver station call. Assume Blue...
        DriverStation.refreshData();

        Optional<Alliance> tmpAlliance = DriverStation.getAlliance();
        Alliance alliance;

        if (tmpAlliance.isPresent()) {
            System.out.print("Alliance is reported as ");
            alliance = tmpAlliance.get();
        } else {
            System.out.print("Alliance is NOT reported. Defaulting to ");
            alliance = Alliance.Blue;
        }

        robotState.setAlliance(alliance);
        System.out.println(alliance == Alliance.Blue ? "Blue" : "Red");
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

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }


}
