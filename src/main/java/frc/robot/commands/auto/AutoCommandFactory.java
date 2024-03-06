package frc.robot.commands.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
    //    middle side
    final ArrayList<ChoreoTrajectory> note_speaker_3 = Choreo.getTrajectoryGroup("3_note_speaker");
    final ArrayList<ChoreoTrajectory> middle_far_amp = Choreo.getTrajectoryGroup("middle_far_amp");
    final ArrayList<ChoreoTrajectory> middle_far_middle_amp = Choreo.getTrajectoryGroup("middle_far_middle_amp");
    final ArrayList<ChoreoTrajectory> middle_far_middle = Choreo.getTrajectoryGroup("middle_far_middle");
    final ArrayList<ChoreoTrajectory> middle_far_middle_source = Choreo.getTrajectoryGroup("middle_far_middle_source");
    final ArrayList<ChoreoTrajectory> middle_far_source = Choreo.getTrajectoryGroup("middle_far_source");

    // amp side
    final ArrayList<ChoreoTrajectory> amp_side_amp = Choreo.getTrajectoryGroup("amp_side_amp");
    final ArrayList<ChoreoTrajectory> amp_side_middle = Choreo.getTrajectoryGroup("amp_side_middle");
    final ArrayList<ChoreoTrajectory> amp_side_source = Choreo.getTrajectoryGroup("amp_side_source");
    final ArrayList<ChoreoTrajectory> far_amp_side_left = Choreo.getTrajectoryGroup("far_amp_side_left");
    final ArrayList<ChoreoTrajectory> far_amp_middle_left = Choreo.getTrajectoryGroup("far_amp_middle_left");
    final ArrayList<ChoreoTrajectory> far_amp_middle = Choreo.getTrajectoryGroup("far_amp_middle");

    //    source side
    final ArrayList<ChoreoTrajectory> source_side_source = Choreo.getTrajectoryGroup("source_side_source");
    final ArrayList<ChoreoTrajectory> source_side_middle = Choreo.getTrajectoryGroup("source_side_middle");
    final ArrayList<ChoreoTrajectory> source_side_amp = Choreo.getTrajectoryGroup("source_side_amp");
    final ArrayList<ChoreoTrajectory> far_source_right = Choreo.getTrajectoryGroup("far_source_right");
    final ArrayList<ChoreoTrajectory> far_source_middle_right = Choreo.getTrajectoryGroup("far_source_middle_right");
    final ArrayList<ChoreoTrajectory> source_far_middle = Choreo.getTrajectoryGroup("source_far_middle");


    private double timestamp = 0;


    public AutoCommandFactory(DrivetrainBase drivetrainBase, Shooter shooter) {
//        System.out.println("Traj pt1: " + note_speaker_3.get(0));
        this.drivetrain = drivetrainBase;
        this.shooter = shooter;
        m_state = RobotState.getInstance();
        drivetrainBase.setHeadingCorrectionTrue();
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


        return Commands.sequence(new ParallelCommandGroup(Commands.print("Start X PID error: " + xController.getPositionError()),
            Choreo.choreoSwerveCommand(
                trajectory,
                drivetrain::getPose,
                xController,
                yController,
                rotationController,
                (ChassisSpeeds speeds) -> drivetrain.drive(speeds, false, 1),
                m_state::isAllianceRed,
                drivetrain
            ), Commands.repeatingSequence(Commands.print("X PID error: " + xController.getPositionError()),
            Commands.print("Trandslation error: " + (xController.getSetpoint() - drivetrain.getPose().getX())))));
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
        System.out.println("getting simple_2m");
//        return Commands.sequence(startAutoSequence(Choreo.getTrajectory("simple_2m")),
//            Commands.waitSeconds(1),autoSequence(Choreo.getTrajectory("back_2m")));
        return Commands.sequence(startAutoSequence(Choreo.getTrajectory("straight_2m")),
            Commands.waitSeconds(1), autoSequence(Choreo.getTrajectory("back_2m")));

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

    public Command ampSideFarMiddle() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(far_amp_middle.get(0)),
            autoSequence(far_amp_middle.get(1)),
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
            autoSequence(source_side_amp.get(1)),
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

    public Command sourceFarMiddle() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(source_far_middle.get(0)),
            autoSequence(source_far_middle.get(1)),
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

    public Command middleFarAmp() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(middle_far_amp.get(0)),
            autoSequence(middle_far_amp.get(1)),
            new Shoot(shooter));
    }

    public Command middleFarMiddleAmp() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(middle_far_middle_amp.get(0)),
            autoSequence(middle_far_middle_amp.get(1)),
            new Shoot(shooter));
    }

    public Command middleFarMiddle() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(middle_far_middle.get(0)),
            autoSequence(middle_far_middle.get(1)),
            new Shoot(shooter));
    }

    public Command middleFarMiddleSource() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(middle_far_middle_source.get(0)),
            autoSequence(middle_far_middle_source.get(1)),
            new Shoot(shooter));
    }

    public Command middleFarSource() {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(middle_far_source.get(0)),
            autoSequence(middle_far_source.get(1)),
            new Shoot(shooter));
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
        private SendableChooser<Boolean> ampFar = new SendableChooser<>();
        private SendableChooser<Boolean> ampMiddleFar = new SendableChooser<>();
        private SendableChooser<Boolean> farMiddle = new SendableChooser<>();
        private SendableChooser<Boolean> sourceFar = new SendableChooser<>();
        private SendableChooser<Boolean> sourceMiddleFar = new SendableChooser<>();

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

            ampFar.setDefaultOption("No", false);
            ampFar.addOption("Yes", true);

            ampMiddleFar.setDefaultOption("No", false);
            ampMiddleFar.addOption("Yes", true);

            farMiddle.setDefaultOption("No", false);
            farMiddle.addOption("Yes", true);

            sourceMiddleFar.setDefaultOption("No", false);
            sourceMiddleFar.addOption("Yes", true);

            sourceFar.setDefaultOption("No", false);
            sourceFar.addOption("Yes", true);

            ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Starting Position?", startingPosition).withPosition(0, 0);
            ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Amp?", ampNote).withPosition(1, 1);
            ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Middle?", middleNote).withPosition(1, 2);
            ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Source?", sourceNote).withPosition(1, 3);
            ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Amp Far?", ampFar).withPosition(2, 1);
            ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Amp Middle Far?", ampMiddleFar).withPosition(2, 2);
            ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Far Middle?", farMiddle).withPosition(2, 3);
            ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Source Middle Far?", sourceMiddleFar).withPosition(2, 4);
            ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Source Far?", sourceFar).withPosition(2, 5);

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
                if (ampFar.getSelected()) {
                    fullRoutine.add(autoCommandFactory.ampSideFarLeft());
                }
                if (ampMiddleFar.getSelected()) {
                    fullRoutine.add(autoCommandFactory.ampSideFarMiddleLeft());
                }
                if (farMiddle.getSelected()) {
                    fullRoutine.add(autoCommandFactory.ampSideFarMiddle());
                }
            } else if (startingPos[startingPosition.getSelected()].equals("middle")) {
                if (ampNote.getSelected()) {
                    fullRoutine.add(autoCommandFactory.middleSideAmpNote());
                }
                if (middleNote.getSelected()) {
                    fullRoutine.add(autoCommandFactory.middleSideMiddleNote());
                }
                if (sourceNote.getSelected()) {
                    fullRoutine.add(autoCommandFactory.middleSideSourceNote());
                }
                if (ampFar.getSelected()) {
                    fullRoutine.add(autoCommandFactory.middleFarAmp());
                }
                if (ampMiddleFar.getSelected()) {
                    fullRoutine.add(autoCommandFactory.middleFarMiddleAmp());
                }
                if (farMiddle.getSelected()) {
                    fullRoutine.add(autoCommandFactory.middleFarMiddle());
                }
                if (sourceMiddleFar.getSelected()) {
                    fullRoutine.add(autoCommandFactory.middleFarMiddleSource());
                }
                if (sourceFar.getSelected()) {
                    fullRoutine.add(autoCommandFactory.middleFarSource());
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
                if (sourceFar.getSelected()) {
                    fullRoutine.add(autoCommandFactory.sourceSideFarRight());
                }
                if (sourceMiddleFar.getSelected()) {
                    fullRoutine.add(autoCommandFactory.sourceSideFarMiddleRight());
                }
                if (farMiddle.getSelected()) {
                    fullRoutine.add(autoCommandFactory.sourceFarMiddle());
                }
            }
            Command[] commandsArray = fullRoutine.toArray(new Command[0]);

            return Commands.sequence(commandsArray);

        }
    }
}
