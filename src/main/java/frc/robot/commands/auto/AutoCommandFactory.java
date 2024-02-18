package frc.robot.commands.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
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

import java.util.ArrayList;

public class AutoCommandFactory {
    final RobotState robotState;
    final DrivetrainBase drivetrain;
    final Shooter shooter;
    final Shoot shoot;
    final ArrayList<ChoreoTrajectory> note_speaker_3 = Choreo.getTrajectoryGroup("3_note_speaker");
    final ArrayList<ChoreoTrajectory> far_side = Choreo.getTrajectoryGroup("far_side");


    public AutoCommandFactory(DrivetrainBase drivetrainBase, Shooter shooter, Shoot shoot) {
        System.out.println("Traj pt1: " + note_speaker_3.get(0));
        this.shoot = shoot;
        this.drivetrain = drivetrainBase;
        this.shooter = shooter;
        robotState = RobotState.getInstance();
    }

    public Command setPoseToTrajectoryStart(ChoreoTrajectory trajectory) {
        boolean reflectField = !robotState.isAllianceBlue();
        Pose2d initialPose = CrescendoField.remapPose(trajectory.getInitialPose(), robotState.isAllianceBlue());

        System.out.println("Setting up trajectory " + trajectory + " for " + (reflectField ? "Red" : "Blue") + " alliance");
        System.out.println("Starting pose = " + initialPose);
        return Commands.runOnce(() -> drivetrain.resetOdometry(initialPose));
    }

    public Command buildChoreoCommand(ChoreoTrajectory trajectory) {
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

    public Command startAutoSequence(ChoreoTrajectory trajectory) {
        return Commands.sequence(setPoseToTrajectoryStart(trajectory), buildChoreoCommand(trajectory));
    }

    public Command autoSequence(ChoreoTrajectory trajectory) {
        return buildChoreoCommand(trajectory);
    }

    public Command simple_2m() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(Choreo.getTrajectory("simple_2m")),
            autoSequence(Choreo.getTrajectory("new2")),
        new Shoot(shooter));
    }

    public Command fourNoteAmp() {
        return startAutoSequence(Choreo.getTrajectory("four_note_w_amp"));
    }

    public Command threeNoteSpeakerPart(int p) {
        return Commands.sequence(autoSequence(note_speaker_3.get(p - 1)),
            new InstantCommand(() ->
            System.out.println("Transformation pt" + p + new Transform2d(note_speaker_3.get(p - 1).getFinalPose(), drivetrain.getPose()))),
        new InstantCommand(() ->
            System.out.println("Vision Transformation pt" + p + new Transform2d( drivetrain.getPose(), robotState.getVisionPose()))));

    }

    public Command farSidePart(int p) {
        return autoSequence(far_side.get(p - 1));

    }

    public Command farSide() {
        return Commands.sequence(new Shoot(shooter),
//            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            setPoseToTrajectoryStart(far_side.get(0)),
            farSidePart(1)
//            farSidePart(2),
//            new Shoot(shooter)
            );

    }

    public Command testAuto() {
        return Commands.sequence(threeNoteSpeakerPart(3), threeNoteSpeakerPart(4));
    }

    public Command threeNoteSpeaker() {
        if (Toggles.useShooter && Toggles.useIntake) {
            return Commands.sequence(
                new Shoot(shooter),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
                setPoseToTrajectoryStart(note_speaker_3.get(2)),
                threeNoteSpeakerPart(3),
                threeNoteSpeakerPart(4),
                new InstantCommand(() -> System.out.println(new Transform2d())),
                new Shoot(shooter),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
                threeNoteSpeakerPart(1),
                threeNoteSpeakerPart(2),
                new Shoot(shooter),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
                threeNoteSpeakerPart(5),
                threeNoteSpeakerPart(6),
                new Shoot(shooter)
            );
        } else {
            return Commands.sequence(
                threeNoteSpeakerPart(1),
                threeNoteSpeakerPart(2),
                threeNoteSpeakerPart(3),
                threeNoteSpeakerPart(4)
            );
        }
    }

    public Command threeNoteSpeakerv3(){
        return Commands.sequence(new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            setPoseToTrajectoryStart(note_speaker_3.get(2)),
            threeNoteSpeakerPart(1),
            threeNoteSpeakerPart(2),
            new InstantCommand(() -> System.out.println(new Transform2d())),
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            threeNoteSpeakerPart(5),
            threeNoteSpeakerPart(6),
            new Shoot(shooter));
    }

    public Command threeNoteSpeakerv2() {
        if (Toggles.useShooter) {
            return Commands.sequence(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SPEAKER_SHOOTING)),
                threeNoteSpeakerPart(1),
                threeNoteSpeakerPart(2),
                threeNoteSpeakerPart(3),
                threeNoteSpeakerPart(4),
                threeNoteSpeakerPart(5),
                threeNoteSpeakerPart(6)
            );
        } else {
            return Commands.sequence(
                threeNoteSpeakerPart(1),
                threeNoteSpeakerPart(2),
                threeNoteSpeakerPart(3),
                threeNoteSpeakerPart(4),
                threeNoteSpeakerPart(5),
                threeNoteSpeakerPart(6)
            );
        }
    }
}
