package frc.robot.commands.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Swerve;

import frc.robot.RobotState;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.CrescendoField;


public class AutoCommands extends Command {

    final RobotState robotState;
    DrivetrainBase drivetrain;
    ShooterSubsystem shooterSubsystem;
    IntakeSubSystem intakeSubSystem;

    Pose2d initialPose;
    ChoreoTrajectory trajectory;
    boolean reflectField;


    public AutoCommands(DrivetrainBase drivetrainBase, ShooterSubsystem shooterSubsystem, IntakeSubSystem intakeSubSystem) {
        this.drivetrain = drivetrainBase;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubSystem = intakeSubSystem;
        robotState = RobotState.getInstance();
    }

    @Override
    public void initialize() {
        System.out.println("Auto INIT");
    }

    public Command setUpPose(String trajectoryName) {
        //        TODO need a better way of getting trajectory name
        trajectory = Choreo.getTrajectory(trajectoryName);
        reflectField = !robotState.isAllianceBlue();
        initialPose = CrescendoField.remapPose(trajectory.getInitialPose(), robotState.isAllianceBlue());


        System.out.println("Setting up Alliance! " + (reflectField ? "Red" : "Blue") + " alliance");
        System.out.println("Starting pose = " + initialPose);
        return Commands.runOnce(() -> drivetrain.resetOdometry(initialPose));
    }


    public Command buildChoreoCommand() {

        boolean openLoop = Swerve.openLoopAuto;

        System.out.println("Control is " + (openLoop ? "open loop" : "pid controlled"));

        PIDController xController = new PIDController(openLoop ? 0 : Swerve.xPidKp,
            openLoop ? 0 : Swerve.xPidKi,
            openLoop ? 0 : Swerve.xPidKd
        );

        PIDController yController = new PIDController(openLoop ? 0 : Swerve.yPidKp,
            openLoop ? 0 : Swerve.yPidKi,
            openLoop ? 0 : Swerve.yPidKd
        );

        PIDController rotationController = new PIDController(openLoop ? 0 : Swerve.rotPidKp,
            openLoop ? 0 : Swerve.rotPidKi,
            openLoop ? 0 : Swerve.rotPidKd
        );


        return Choreo.choreoSwerveCommand(
            trajectory,
            drivetrain::getPose,
            xController,
            yController,
            rotationController,
            (ChassisSpeeds speeds) -> drivetrain.drive(speeds, false, 1),
            () -> reflectField,
            drivetrain
        );

    }


    public Command test2m() {
        return Commands.sequence(setUpPose("simple_2m"), buildChoreoCommand());
    }

    public Command fourNoteAmp() {
        return Commands.sequence(setUpPose("four_note_w_amp"), buildChoreoCommand());
    }

    public Command threeNoteSpeakerpt1() {
        return Commands.sequence(setUpPose("3_note_speaker_pt1"), buildChoreoCommand());
    }

    public Command threeNoteSpeakerpt2() {
        return Commands.sequence(setUpPose("3_note_speaker_pt2"), buildChoreoCommand());
    }

    public Command threeNoteSpeakerpt3() {
        return Commands.sequence(setUpPose("3_note_speaker_pt3"), buildChoreoCommand());
    }

    public Command threeNoteSpeakerpt4() {
        return Commands.sequence(setUpPose("3_note_speaker_pt4"), buildChoreoCommand());
    }

    public Command threeNoteSpeakerpt5() {
        return Commands.sequence(setUpPose("3_note_speaker_pt5"), buildChoreoCommand());
    }

    public Command threeNoteSpeakerpt6() {
        return Commands.sequence(setUpPose("3_note_speaker_pt6"), buildChoreoCommand());
    }
// wrong for now
    public Command threeNoteSpeaker() {
        return Commands.sequence(shooterSubsystem.autoShoot(), threeNoteSpeakerpt1(),
            intakeSubSystem.autoIntake(), threeNoteSpeakerpt2(),
            shooterSubsystem.autoShoot(), threeNoteSpeakerpt3(),
            intakeSubSystem.autoIntake(), threeNoteSpeakerpt4(),
            shooterSubsystem.autoShoot());
    }

    public Command threeNoteSpeakerv2() {
        return Commands.sequence(shooterSubsystem.autoShoot(),
            threeNoteSpeakerpt1(),
            threeNoteSpeakerpt2(),
            threeNoteSpeakerpt3(),
            threeNoteSpeakerpt4(),
            threeNoteSpeakerpt5(),
            threeNoteSpeakerpt6());
    }
}
