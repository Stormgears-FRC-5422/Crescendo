// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ButtonBoard;
import frc.robot.Constants.Choreo;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Toggles;
import frc.robot.commands.DriveToNote;
import frc.robot.commands.JoyStickDrive;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.auto.AutoCommandFactory;
import frc.robot.commands.climb.Climbing;
import frc.robot.commands.climb.EmergencyStop;
import frc.robot.commands.climb.Home;
import frc.robot.commands.climb.SimpleGotoDegrees;
import frc.robot.commands.shoot.*;
import frc.robot.joysticks.CrescendoJoystick;
import frc.robot.joysticks.CrescendoJoystickFactory;
import frc.robot.joysticks.IllegalJoystickTypeException;
import frc.robot.subsystems.*;
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
    private StatusLights statusLights;
    private NavX navX;
    private Shooter shooter;
    private VisionSubsystem visionSubsystem;
    private Climber climber;


    // **********
    // Commands
    // **********
    private LoggedDashboardChooser<Command> autoChooser;
    private AutoCommandFactory autoCommandFactory;
    private AutoCommandFactory.AutoSelector autoSelector;
    private Command m_noChooserCommand;
    private Shoot shoot;
    private AmpShoot ampShoot;
    private GroundPickup groundPickup;
    private DiagnosticShooterIntake diagnosticShooterIntake;
    private Outtake outtake;
    private SourceIntake sourceIntake;
    private Climbing climbing;
    private SimpleGotoDegrees gotoAmpShootPosition;
    private SimpleGotoDegrees gotoStowPosition;
    private SimpleGotoDegrees gotoClimbStartPosition;
    private Home home;
    private EmergencyStop emergencyStop;


    // **********
    // Fields
    // **********
    final RobotState robotState;

    // **********
    // Control
    // **********
    CrescendoJoystick joystick;
    CrescendoJoystick joystick2;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() throws IllegalDriveTypeException, IllegalJoystickTypeException {
        System.out.println("[Init] RobotContainer");
        robotState = RobotState.getInstance();

        // start somewhere
        Pose2d initialPose = new Pose2d(ButtonBoard.initPoseX, ButtonBoard.initPoseY,
            Rotation2d.fromDegrees(ButtonBoard.initPoseDegrees));
        robotState.setPose(initialPose);

        if (Toggles.useVision) {
            visionSubsystem = new VisionSubsystem("limelight-note");
        }

        if (Toggles.useDrive) {
            System.out.println("Create drive type " + Drive.driveType);
            drivetrain = DrivetrainFactory.getInstance(Drive.driveType);
            if (Toggles.useController) {
                System.out.println("Making 1st joystick!");

                joystick = CrescendoJoystickFactory.getInstance(ButtonBoard.driveJoystick, ButtonBoard.driveJoystickPort);
                JoyStickDrive driveWithJoystick = new JoyStickDrive(drivetrain, joystick);
                drivetrain.setDefaultCommand(driveWithJoystick);
            }
            if (Toggles.useSecondXbox) {
                System.out.println("Making 2nd joystick!");
                joystick2 = CrescendoJoystickFactory.getInstance(ButtonBoard.joystick2, ButtonBoard.secondJoystickPort);
            }
        }

        if (Toggles.useShooter && Toggles.useIntake) {
            shooter = new Shooter();
            shoot = new Shoot(shooter);
            diagnosticShooterIntake = new DiagnosticShooterIntake(shooter);
            groundPickup = new GroundPickup(shooter);
            ampShoot = new AmpShoot(shooter);
            outtake = new Outtake(shooter);
            sourceIntake = new SourceIntake(shooter);
        }

        if (Toggles.useNavX && !Drive.driveType.equals("YagslDrive")) {
            System.out.println("Create NavX");
            navX = new NavX();
        }

        if (Toggles.useStatusLights) {
            statusLights = new StatusLights();
        }

        if (Toggles.useDrive) {
            autoCommandFactory = new AutoCommandFactory(drivetrain, shooter);
            autoSelector = new AutoCommandFactory.AutoSelector(drivetrain, shooter);
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
                m_noChooserCommand = switch (Choreo.path.toLowerCase()) {
                    case "simple_2m" -> autoCommandFactory.simple_2m();
                    case "four_note_w_amp" -> autoCommandFactory.fourNoteAmp();
                    case "3_note_speaker" -> autoCommandFactory.threeNoteSpeaker();
                    case "3_note_speaker_v2" -> autoCommandFactory.threeNoteSpeakerv2();
                    case "testauto" -> autoCommandFactory.testAuto();
//                    case "farside" -> autoCommandFactory.farSide();
                    case "3_note_speaker_v3" -> autoCommandFactory.threeNoteSpeakerv3();
                    default -> autoCommandFactory.startAutoSequence(
                        Choreo.path.isBlank() ? "simple_2m" : Choreo.path);
                };
            }
        }

        if (Toggles.useClimber) {
            climber = new Climber();
            climbing = new Climbing(climber);
            home = new Home(climber);
            emergencyStop = new EmergencyStop(climber);
            // TODO - these commands assume a direction to reach the target. We should add safeties to confirm
            // we don't end up going the wrong direction and destroy a note in the process.
            // ideally the climber has knowledge of all of the positions, perhaps through a set of enumerated
            // locations and could pick the correct path in all cases.
            gotoAmpShootPosition = new SimpleGotoDegrees(climber, Constants.Climber.ampShootDegrees,
                Climber.Direction.FORWARD);
            gotoStowPosition = new SimpleGotoDegrees(climber, Constants.Climber.stowDegrees,
                Climber.Direction.REVERSE);
            gotoClimbStartPosition = new SimpleGotoDegrees(climber, Constants.Climber.climbStartDegrees,
                Climber.Direction.FORWARD);
        }

        // Configure the trigger bindings
        configureBindings();

        System.out.println("[DONE] RobotContainer");
    }

    public void updateAlliance() {
        DriverStation.refreshData();
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            robotState.setAlliance(alliance.get() == Alliance.Blue ? RobotState.StateAlliance.BLUE : RobotState.StateAlliance.RED);
        } else {
            robotState.setAlliance(RobotState.StateAlliance.MISSING);
        }
    }

    public void resetInitialPose() {
        if (robotState.isAllianceMissing()) return;

        Pose2d initialPose = new Pose2d(ButtonBoard.initPoseX, ButtonBoard.initPoseY,
            Rotation2d.fromDegrees(ButtonBoard.initPoseDegrees));

        initialPose = CrescendoField.remapPose(initialPose, robotState.getAlliance());

        drivetrain.declarePoseIsNow(initialPose);
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
        new Trigger(() -> joystick.driveNote()).onTrue(Commands.parallel(new DriveToNote(drivetrain, visionSubsystem), new GroundPickup(shooter)));
        System.out.println("[Init] configureBindings");

        if (Toggles.useSysId) {
//            new Trigger(() -> joystick.diagnosticShooterIntake()).onTrue(drivetrain.getSysIdCommand()); //down arrow
            new Trigger(() -> joystick.diagnosticShooterIntake()).onTrue(drivetrain.getQuasForwardCommand()); //down arrow
            new Trigger(() -> joystick.shooterAmp()).onTrue(drivetrain.getQuasBackwardCommand()); //x button
            new Trigger(() -> joystick.outtake()).onTrue(drivetrain.getDynamicForwardCommand()); //up arrow
            new Trigger(() -> joystick.shooterIntake()).onTrue(drivetrain.getDynamicBackwardCommand()); //y button
            new Trigger(() -> joystick.zeroWheels()).onTrue(new InstantCommand(() -> drivetrain.zeroWheels()));
        }

        if (Toggles.useIntake && Toggles.useShooter && !Toggles.useSysId && Toggles.useClimber) {
            new Trigger(() -> joystick.zeroGyro()).onTrue(new InstantCommand(() -> drivetrain.resetOrientation())
            );
            new Trigger(() -> joystick.shooter()).onTrue(shoot);

            new Trigger
                (() -> joystick.intake()).onTrue(
                groundPickup
                    .andThen(new RumbleCommand(joystick, 1.0).unless(() -> !robotState.isUpperSensorTriggered())));
            new Trigger(() -> joystick.diagnosticShooterIntake()).onTrue(diagnosticShooterIntake);
            new Trigger(() -> joystick.shooterAmp()).onTrue(Commands.sequence(gotoAmpShootPosition,
                ampShoot, gotoStowPosition));
            new Trigger(() -> joystick.outtake()).onTrue(outtake);
            new Trigger(() -> joystick.shooterIntake()).onTrue(sourceIntake);
            new Trigger(() -> joystick.zeroWheels()).onTrue(new InstantCommand(()->drivetrain.zeroWheels()));
            new Trigger(() -> joystick.climb()).onTrue(climbing);
            new Trigger(() -> joystick.home()).onTrue(home);
            new Trigger(() -> joystick.climberEmergencyStop()).onTrue(emergencyStop);
            new Trigger(() -> joystick.ampPosition()).onTrue(new SimpleGotoDegrees(climber, Constants.Climber.ampShootDegrees,
                Climber.Direction.REVERSE));

        }


        if (Toggles.useSecondXbox) {
            System.out.println("Configure Second Joystick");
            new Trigger(() -> joystick2.zeroGyro()).onTrue(new InstantCommand(() -> drivetrain.resetOrientation()));
            new Trigger(() -> joystick2.shooter()).onTrue(shoot);
            new Trigger(() -> joystick2.intake()).onTrue(groundPickup);
            new Trigger(() -> joystick2.diagnosticShooterIntake()).onTrue(diagnosticShooterIntake);
            new Trigger(() -> joystick2.shooterAmp()).onTrue(ampShoot);
            new Trigger(() -> joystick2.outtake()).onTrue(outtake);
            new Trigger(() -> joystick2.shooterIntake()).onTrue(sourceIntake);
        }


        System.out.println("[DONE] configureBindings");
    }


    public Command getAutonomousCommand() {
        if (Toggles.useAutoSelector) {
            return autoSelector.buildAuto();
        } else if (Toggles.useAutoChooser && Toggles.useAdvantageKit) {
            System.out.println("AutoChooser command: " + autoChooser.get());
            return autoChooser.get();
        } else {
            return m_noChooserCommand;
        }
    }

}
