package frc.robot.commands.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Toggles;

import frc.robot.RobotState;
import frc.robot.ShuffleboardConstants;
import frc.robot.commands.shoot.Shoot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.CrescendoField;

import java.util.ArrayList;

public class AutoCommandFactory {
    final RobotState m_state;
    final DrivetrainBase drivetrain;
    final Shooter shooter;
    final ArrayList<ChoreoTrajectory> note_speaker_3 = Choreo.getTrajectoryGroup("3_note_speaker");
    //    final ArrayList<ChoreoTrajectory> far_side = Choreo.getTrajectoryGroup("far_side");
    final ArrayList<ChoreoTrajectory> amp_side_amp = Choreo.getTrajectoryGroup("amp_side_amp");
    final ArrayList<ChoreoTrajectory> amp_side_middle = Choreo.getTrajectoryGroup("amp_side_middle");
    final ArrayList<ChoreoTrajectory> amp_side_source = Choreo.getTrajectoryGroup("amp_side_source");
    final ArrayList<ChoreoTrajectory> far_amp_side_left = Choreo.getTrajectoryGroup("far_amp_side_left");
    final ArrayList<ChoreoTrajectory> far_amp_middle_left = Choreo.getTrajectoryGroup("far_amp_middle_left");

    final ArrayList<ChoreoTrajectory> source_side_source = Choreo.getTrajectoryGroup("source_side_source");
    final ArrayList<ChoreoTrajectory> source_side_middle = Choreo.getTrajectoryGroup("source_side_middle");
    final ArrayList<ChoreoTrajectory> source_side_amp = Choreo.getTrajectoryGroup("source_side_amp");
    final ArrayList<ChoreoTrajectory> far_source_right = Choreo.getTrajectoryGroup("far_source_right");
    final ArrayList<ChoreoTrajectory> far_source_middle_right = Choreo.getTrajectoryGroup("far_source_middle_right");


    public AutoCommandFactory(DrivetrainBase drivetrainBase, Shooter shooter) {
        System.out.println("Traj pt1: " + note_speaker_3.get(0));
        this.drivetrain = drivetrainBase;
        this.shooter = shooter;
        m_state = RobotState.getInstance();
    }

    public Command setPoseToTrajectoryStart(ChoreoTrajectory trajectory) {
        return Commands.runOnce(() -> {
            // TODO - ultimately we want this initial pose to come from vision
            Pose2d initialPose = CrescendoField.remapPose(trajectory.getInitialPose(), m_state.getAlliance());
            System.out.println("Setting up trajectory " + trajectory + " for " + m_state.getAlliance() + " alliance");
            System.out.println("Asserting starting pose = " + initialPose);

            drivetrain.declarePoseIsNow(initialPose);
        });
    }

    public Command buildChoreoCommand(ChoreoTrajectory trajectory) {
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
            m_state::isAllianceRed,
            drivetrain
        );
    }

    public Command startAutoSequence(String trajectoryName) {
        return startAutoSequence(Choreo.getTrajectory(trajectoryName));
    }

    public Command startAutoSequence(ChoreoTrajectory trajectory) {
        return Commands.sequence(setPoseToTrajectoryStart(trajectory), buildChoreoCommand(trajectory));
    }

    public Command autoSequence(ChoreoTrajectory trajectory) {
        return buildChoreoCommand(trajectory);
    }

    public Command simple_2m() {
//        return Commands.sequence(
//            new Shoot(shooter),
//            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
//            startAutoSequence(Choreo.getTrajectory("simple_2m")),
//            autoSequence(Choreo.getTrajectory("new2")),
//        new Shoot(shooter));
        return startAutoSequence(Choreo.getTrajectory("simple_2m"));
    }

    public Command ampSideAmpNote() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(amp_side_amp.get(0)),
            autoSequence(amp_side_amp.get(1)),
            new Shoot(shooter));
    }

    public Command ampSideMiddleNote() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(amp_side_middle.get(0)),
            autoSequence(amp_side_middle.get(1)),
            new Shoot(shooter));
    }

    public Command ampSideSourceNote() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(amp_side_source.get(0)),
            autoSequence(amp_side_source.get(1)),
            new Shoot(shooter));
    }

    public Command ampSideFarLeft() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(far_amp_side_left.get(0)),
            autoSequence(far_amp_side_left.get(1)),
            new Shoot(shooter));
    }

    public Command ampSideFarMiddleLeft() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(far_amp_middle_left.get(0)),
            autoSequence(far_amp_middle_left.get(1)),
            new Shoot(shooter));
    }

    public Command middleSideMiddleNote() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(note_speaker_3.get(0)),
            autoSequence(note_speaker_3.get(1)),
            new Shoot(shooter));
    }

    public Command middleSideAmpNote() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(note_speaker_3.get(2)),
            autoSequence(note_speaker_3.get(3)),
            new Shoot(shooter));
    }

    public Command middleSideSourceNote() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(note_speaker_3.get(4)),
            autoSequence(note_speaker_3.get(5)),
            new Shoot(shooter));
    }

    public Command sourceSideSourceNote() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(source_side_source.get(0)),
            autoSequence(source_side_source.get(1)),
            new Shoot(shooter));
    }

    public Command sourceSideMiddleNote() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(source_side_middle.get(0)),
            autoSequence(source_side_middle.get(1)),
            new Shoot(shooter));
    }

    public Command sourceSideAmpNote() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(source_side_amp.get(0)),
            autoSequence(source_side_source.get(1)),
            new Shoot(shooter));
    }

    public Command sourceSideFarRight() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(far_source_right.get(0)),
            autoSequence(far_source_right.get(1)),
            new Shoot(shooter));
    }

    public Command sourceSideFarMiddleRight() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(far_source_middle_right.get(0)),
            autoSequence(far_source_middle_right.get(1)),
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
                System.out.println("Vision Transformation pt" + p + new Transform2d(drivetrain.getPose(), m_state.getVisionPose()))));

    }

//    public Command farSidePart(int p) {
//        return autoSequence(far_side.get(p - 1));
//
//    }

//    public Command farSide() {
//        return Commands.sequence(new Shoot(shooter),
////            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
//            setPoseToTrajectoryStart(far_side.get(0)),
//            farSidePart(1)
////            farSidePart(2),
////            new Shoot(shooter)
//        );
//
//    }

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

    public Command threeNoteSpeakerv3() {
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


    public static class AutoSelector {
        String[] startingPos = {"amp", "middle", "source"};
        AutoCommandFactory autoCommandFactory;

        private SendableChooser<Integer> startingPosition = new SendableChooser<>();
        private SendableChooser<Boolean> ampNote = new SendableChooser<>();
        private SendableChooser<Boolean> middleNote = new SendableChooser<>();
        private SendableChooser<Boolean> sourceNote = new SendableChooser<>();
        private SendableChooser<Boolean> farLeft = new SendableChooser<>();
        private SendableChooser<Boolean> farMiddleLeft = new SendableChooser<>();
        private SendableChooser<Boolean> farMiddle = new SendableChooser<>();
        private SendableChooser<Boolean> farMiddleRight = new SendableChooser<>();
        private SendableChooser<Boolean> farRight = new SendableChooser<>();

        public AutoSelector(DrivetrainBase drivetrainBase, Shooter shooter) {

            autoCommandFactory = new AutoCommandFactory(drivetrainBase, shooter);

            for (int i = 0; i < 3; i++) {
                if (i == 0) {
                    startingPosition.setDefaultOption(startingPos[i], i);
                    continue;
                }
                startingPosition.addOption(startingPos[i], i);
            }


            ampNote.setDefaultOption("No", false);
            ampNote.addOption("Yes", true);

            middleNote.setDefaultOption("No", false);
            middleNote.addOption("Yes", true);

            sourceNote.setDefaultOption("No", false);
            sourceNote.addOption("Yes", true);

            farLeft.setDefaultOption("No", false);
            farLeft.addOption("Yes", true);

            farMiddleLeft.setDefaultOption("No", false);
            farMiddleLeft.addOption("Yes", true);

            farMiddle.setDefaultOption("No", false);
            farMiddle.addOption("Yes", true);

            farMiddleRight.setDefaultOption("No", false);
            farMiddleRight.addOption("Yes", true);

            farRight.setDefaultOption("No", false);
            farRight.addOption("Yes", true);

            ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Starting Position?", startingPosition).withPosition(0, 0);
            ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Amp?", ampNote).withPosition(1, 1);
            ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Middle?", middleNote).withPosition(1, 2);
            ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Source?", sourceNote).withPosition(1, 3);
            ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Far Left?", farLeft).withPosition(2, 1);
            ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Far Middle Left?", farMiddleLeft).withPosition(2, 2);
            ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Far Middle?", farMiddle).withPosition(2, 3);
            ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Far Middle Right?", farMiddleRight).withPosition(2, 4);
            ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Far Right?", farRight).withPosition(2, 5);

        }

        public Command buildAuto() {
            ArrayList<Command> fullRoutine = new ArrayList<>();
            if (startingPos[startingPosition.getSelected()].equals("amp")) {
                if (ampNote.getSelected()) {
                    fullRoutine.add(autoCommandFactory.ampSideAmpNote());
                }
                if (middleNote.getSelected()) {
                    fullRoutine.add(autoCommandFactory.ampSideMiddleNote());
                }
                if (sourceNote.getSelected()) {
                    fullRoutine.add(autoCommandFactory.ampSideSourceNote());

                }
                if (farLeft.getSelected()) {
                    fullRoutine.add(autoCommandFactory.ampSideFarLeft());
                }
                if (farMiddleLeft.getSelected()) {
                    fullRoutine.add(autoCommandFactory.ampSideFarMiddleLeft());
                }
            } else if (startingPos[startingPosition.getSelected()].equals("middle")) {
                if (ampNote.getSelected()) {
                    fullRoutine.add(autoCommandFactory.middleSideAmpNote());
                }
                if (middleNote.getSelected()) {
                    fullRoutine.add(autoCommandFactory.middleSideMiddleNote());
                }
                if (middleNote.getSelected()) {
                    fullRoutine.add(autoCommandFactory.middleSideSourceNote());
                }
            } else {
                if (sourceNote.getSelected()) {
                    fullRoutine.add(autoCommandFactory.sourceSideSourceNote());

                }
                if (middleNote.getSelected()) {
                    fullRoutine.add(autoCommandFactory.sourceSideMiddleNote());
                }
                if (ampNote.getSelected()) {
                    fullRoutine.add(autoCommandFactory.sourceSideAmpNote());
                }
                if (farRight.getSelected()) {
                    fullRoutine.add(autoCommandFactory.sourceSideFarRight());
                }
                if (farMiddleRight.getSelected()) {
                    fullRoutine.add(autoCommandFactory.sourceSideFarMiddleRight());
                }
            }
            Command[] commandsArray = fullRoutine.toArray(new Command[0]);

            return Commands.sequence(commandsArray);

        }
    }
}
