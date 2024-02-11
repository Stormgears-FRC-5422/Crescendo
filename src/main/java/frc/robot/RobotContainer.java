// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Drive;
import frc.robot.Constants.ButtonBoard;
import frc.robot.Constants.Toggles;
import frc.robot.Constants.Choreo;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JoyStickDrive;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TestCases;
import frc.robot.commands.auto.AutoCommandFactory;
import frc.robot.joysticks.*;
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
    private DrivetrainBase drivetrain;
    private NavX navX;
    private ShooterSubsystem shooter;
    private IntakeSubSystem intake;

    // **********
    // Commands
    // **********
    private LoggedDashboardChooser<Command> autoChooser;
    private AutoCommandFactory autoCommandFactory;
    private Command m_noChooserCommand;
    private ShooterCommand shooterCommand;
    private IntakeCommand intakeCommand;


    // **********
    // Fields
    // **********
    final RobotState robotState;

    // **********
    // Control
    // **********
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

        autoCommandFactory = new AutoCommandFactory(drivetrain, shooter, intake);
        if (Toggles.useAutoChooser && Toggles.useAdvantageKit) {
            // TODO - we shouldn't hard code these path names here. Not sure the right way to list them
            // probably in a config setting like (String) simple_2m | 4noteAmp | 3noteSpeaker | etc.
            // so we can add things to this list without messing with the code.
            // This gets tricky if the commands might vary based on subsystem availability
            System.out.println("Using AutoChooser");
            autoChooser = new LoggedDashboardChooser<>("Auto Choices");
            autoChooser.addOption("simple_2m", autoCommandFactory.simple_2m());
            autoChooser.addOption("four_note_w_amp", autoCommandFactory.fourNoteAmp());
            autoChooser.addOption("testAuto", autoCommandFactory.testAuto());
            if (Toggles.useShooter && Toggles.useIntake) {
                autoChooser.addOption("3_note_speaker", autoCommandFactory.threeNoteSpeaker());
            }
            if (Toggles.useShooter) {
                autoChooser.addOption("3_note_speaker_v2", autoCommandFactory.threeNoteSpeakerv2());
            }
            autoChooser.addDefaultOption("simple_2m", autoCommandFactory.simple_2m());
        } else {
            System.out.println("NoChooser Command set to " + Choreo.path);
            m_noChooserCommand = switch(Choreo.path.toLowerCase()) {
                case "simple_2m" -> autoCommandFactory.simple_2m();
                case "four_note_w_amp" -> autoCommandFactory.fourNoteAmp();
                case "3_note_speaker" -> autoCommandFactory.threeNoteSpeaker();
                case "3_note_speaker_v2" -> autoCommandFactory.threeNoteSpeakerv2();
                case "testauto" -> autoCommandFactory.testAuto();
                default -> autoCommandFactory.simple_2m();
            };
        }

        System.out.println("[DONE] RobotContainer");
    }

    public void setAlliance() {
        // TODO We don't always get a clear alliance from the driver station call. Assume ButtonBoard.defaultAlliance
        DriverStation.refreshData();

        Optional<Alliance> tmpAlliance = DriverStation.getAlliance();
        Alliance alliance;

        if (tmpAlliance.isPresent()) {
            System.out.print("Alliance is reported as ");
            alliance = tmpAlliance.get();
        } else {
            System.out.print("Alliance is NOT reported. Defaulting to ");
            alliance = switch (ButtonBoard.defaultAlliance.toLowerCase()) {
                case "blue" -> Alliance.Blue;
                case "red" -> Alliance.Red;
                default -> Alliance.Blue;
            };
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

        new Trigger(()-> joystick.zeroGyro()).onTrue(new InstantCommand(()->drivetrain.resetGyro()));

        System.out.println("[DONE] configureBindings");
    }

    public Command getAutonomousCommand() {
        if (Toggles.useAutoChooser && Toggles.useAdvantageKit) {
            System.out.println("AutoChooser command: " + autoChooser.get());
            return autoChooser.get();
        } else {
            return m_noChooserCommand;
        }
    }

    public Command getTestCommand() {
        return new SequentialCommandGroup(
        new TestCases(drivetrain, joystick),
        new ParallelCommandGroup());
  }
}
