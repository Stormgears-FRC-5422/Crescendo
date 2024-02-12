package frc.robot.commands.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Toggles;

import frc.robot.RobotState;
import frc.robot.commands.shoot.Shoot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.CrescendoField;

public class AutoCommandFactory {
    final RobotState robotState;
    final DrivetrainBase drivetrain;
    final Shooter shooter;
    final Shoot shoot;


    public AutoCommandFactory(DrivetrainBase drivetrainBase, Shooter shooter, Shoot shoot) {
        this.shoot = shoot;
        this.drivetrain = drivetrainBase;
        this.shooter = shooter;
        robotState = RobotState.getInstance();
    }

    public Command setPoseToTrajectoryStart(String trajectoryName) {
        // TODO need a better way of getting trajectory name
        ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        boolean reflectField = !robotState.isAllianceBlue();
        Pose2d initialPose = CrescendoField.remapPose(trajectory.getInitialPose(), robotState.isAllianceBlue());

        System.out.println("Setting up trajectory " + trajectoryName + " for " + (reflectField ? "Red" : "Blue") + " alliance");
        System.out.println("Starting pose = " + initialPose);
        return Commands.runOnce(() -> drivetrain.resetOdometry(initialPose));
    }

    public Command buildChoreoCommand(String trajectoryName) {
        ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        boolean reflectField = !robotState.isAllianceBlue();

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

    public Command simpleAutoSequence(String name) {
        return Commands.sequence(setPoseToTrajectoryStart(name), buildChoreoCommand(name));
    }

    public Command simple_2m() {
        return simpleAutoSequence("simple_2m");
    }

    public Command fourNoteAmp() {
        return simpleAutoSequence("four_note_w_amp");
    }

    public Command threeNoteSpeakerpt1() {
        return simpleAutoSequence("3_note_speaker_pt1");
    }

    public Command threeNoteSpeakerpt2() {
        return simpleAutoSequence("3_note_speaker_pt2");
    }

    public Command threeNoteSpeakerpt3() {
        return simpleAutoSequence("3_note_speaker_pt3");
    }

    public Command threeNoteSpeakerpt4() {
        return simpleAutoSequence("3_note_speaker_pt4");
    }

    public Command threeNoteSpeakerpt5() {
        return simpleAutoSequence("3_note_speaker_pt5");
    }

    public Command threeNoteSpeakerpt6() {
        return simpleAutoSequence("3_note_speaker_pt6");
    }

//    public Command testAuto() {
//        return  Commands.sequence(shooterSubsystem.autoShoot(),threeNoteSpeakerpt3(),
//            intakeSubSystem.autoIntake(), threeNoteSpeakerpt4(),
//            shooterSubsystem.autoShoot(), threeNoteSpeakerpt1(),
//            intakeSubSystem.autoIntake(),threeNoteSpeakerpt2(),
//            shooterSubsystem.autoShoot());
//    }

//    public Command testAuto() {return Commands.sequence(threeNoteSpeakerpt3(),
//        threeNoteSpeakerpt4(),threeNoteSpeakerpt1(),threeNoteSpeakerpt2());}
//
    public Command testAuto() {
        return Commands.sequence(threeNoteSpeakerpt3(), threeNoteSpeakerpt4());
    }



    public Command threeNoteSpeaker() {
        if (Toggles.useShooter && Toggles.useIntake) {
            return Commands.sequence(
                new Shoot(shooter),
                    new InstantCommand(()-> shooter.ShooterStateMachine(Shooter.ShooterStates.GROUND_PICKUP)),
                threeNoteSpeakerpt3(),
                 threeNoteSpeakerpt4(),
                new Shoot(shooter),
                new InstantCommand(()-> shooter.ShooterStateMachine(Shooter.ShooterStates.GROUND_PICKUP)),
                threeNoteSpeakerpt1(),
                 threeNoteSpeakerpt2(),
                new Shoot(shooter)
            );
        } else {
            return Commands.sequence(
                threeNoteSpeakerpt1(),
                threeNoteSpeakerpt2(),
                threeNoteSpeakerpt3(),
                threeNoteSpeakerpt4()
            );
        }
    }

    public Command threeNoteSpeakerv2() {
        if (Toggles.useShooter) {
            return Commands.sequence(
                new InstantCommand(()-> shooter.ShooterStateMachine(Shooter.ShooterStates.SPEAKER_SHOOTING)),
                threeNoteSpeakerpt1(),
                threeNoteSpeakerpt2(),
                threeNoteSpeakerpt3(),
                threeNoteSpeakerpt4(),
                threeNoteSpeakerpt5(),
                threeNoteSpeakerpt6()
            );
        } else {
            return Commands.sequence(
                threeNoteSpeakerpt1(),
                threeNoteSpeakerpt2(),
                threeNoteSpeakerpt3(),
                threeNoteSpeakerpt4(),
                threeNoteSpeakerpt5(),
                threeNoteSpeakerpt6()
            );
        }
    }
}
