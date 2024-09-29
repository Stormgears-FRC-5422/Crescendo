package frc.robot.commands.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.*;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Toggles;

import frc.robot.commands.DriveToNote;
import frc.robot.commands.shoot.Shoot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.StormChoreo;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class AutoCommandFactory {
    final RobotState m_state;
    final DrivetrainBase drivetrain;
    final Shooter shooter;
    final VisionSubsystem visionSubsystem;
    final VisionSubsystem visionSubsystemNote;
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
    final ArrayList<ChoreoTrajectory>
        far_source_right = Choreo.getTrajectoryGroup("far_source_right");
    final ArrayList<ChoreoTrajectory> far_source_middle_right = Choreo.getTrajectoryGroup("far_source_middle_right");
    final ArrayList<ChoreoTrajectory> source_far_middle = Choreo.getTrajectoryGroup("source_far_middle");

    private int count = 0;



    public AutoCommandFactory(DrivetrainBase drivetrainBase, Shooter shooter, VisionSubsystem visionSubsystem, VisionSubsystem visionSubsystemNote) {
//        System.out.println("Traj pt1: " + note_speaker_3.get(0));
        this.drivetrain = drivetrainBase;
        this.shooter = shooter;
        this.visionSubsystem = visionSubsystem;
        this.visionSubsystemNote = visionSubsystemNote;
        m_state = RobotState.getInstance();
        drivetrainBase.setHeadingCorrectionTrue();


    }

    public Command setPoseToTrajectoryStart(ChoreoTrajectory trajectory) {
        return Commands.runOnce(() -> {
            if (count == 0 && !RobotState.getInstance().isVisionPoseValid()) {
                // TODO - ultimately we want this initial pose to come from vision
                Pose2d initialPose = CrescendoField.remapPose(trajectory.getInitialPose(), m_state.getAlliance());
                System.out.println("Setting up trajectory " + trajectory + " for " + m_state.getAlliance() + " alliance");
                System.out.println("Asserting starting pose = " + initialPose);

                drivetrain.declarePoseIsNow(initialPose);
                count++;
            } else {
                System.out.println("setPose to trjectory already ran");
            }
        });
    }

    public Command buildChoreoCommand(ChoreoTrajectory trajectory) {
//        Logger.recordOutput("X setpoint", xController.getSetpoint());
//        Logger.recordOutput("Translation X", drivetrain.getPose().getX());
//        Logger.recordOutput("Y setpoint", yController.getSetpoint());
//        Logger.recordOutput("Translation Y", drivetrain.getPose().getY());
//        Logger.recordOutput("Rot setpoint", rotationController.getSetpoint());
//        Logger.recordOutput("Translation Rot", drivetrain.getPose().getRotation().getDegrees());
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

        System.out.println("X P: " + xController.getP());

        return Commands.sequence(new InstantCommand(drivetrain::zeroWheels),
            StormChoreo.choreoSwerveCommand(
                trajectory,
                drivetrain::getPose,
                xController,
                yController,
                rotationController,
                (ChassisSpeeds speeds) -> drivetrain.drive(speeds, false, 1),
                m_state::isAllianceRed,
                drivetrain), new InstantCommand(drivetrain::stopDrive)
        );
    }

    public Command startAutoSequence(String trajectoryName) {
        System.out.println("In startAutoSequence for trajectory : " + trajectoryName);
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
        System.out.println("getting straight_2m_2");
//        return Commands.sequence(startAutoSequence(Choreo.getTrajectory("simple_2m")),
//            Commands.waitSeconds(1),autoSequence(Choreo.getTrajectory("back_2m")));
        return Commands.sequence(startAutoSequence(Choreo.getTrajectory("straight_2m_2")));
//            Commands.waitSeconds(1), autoSequence(Choreo.getTrajectory("back_2m")));

    }

//    public Command commandBuilder(ChoreoTrajectory trajectory, ChoreoTrajectory trajectory2) {
//        return Commands.sequence(
//            new Shoot(shooter),
//            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
//            startAutoSequence(trajectory),
//            new DriveToNote(drivetrain, visionSubsystem),
//            autoSequence(trajectory2),
//            new InstantCommand(() -> drivetrain.setVisionPose(RobotState.getInstance().getVisionPose(visionSubsystem))),
//            new Shoot(shooter));
////    }
//
//    public Command commandBuilder(ChoreoTrajectory trajectory, ChoreoTrajectory trajectory2) {
//        return Commands.sequence(
//            new InstantCommand(() -> drivetrain.setVisionPose(RobotState.getInstance().getVisionPose(visionSubsystem))),
//            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
//            startAutoSequence(trajectory),
//            new DriveToNote(drivetrain, visionSubsystem).withTimeout(1.5),
//            autoSequence(trajectory2),
//        new Shoot(shooter));
//    }

    //    }

    public Command commandBuilder(ChoreoTrajectory trajectory, ChoreoTrajectory trajectory2) {
        return Commands.sequence(
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
//            new InstantCommand(() -> drivetrain.setVisionPose(RobotState.getInstance().getVisionPose())),
            startAutoSequence(trajectory),
            new DriveToNote(drivetrain, visionSubsystemNote),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            autoSequence(trajectory2),
            new Shoot(shooter));
    }

    public Command commandBuilder(ChoreoTrajectory trajectory) {
        return Commands.sequence(
            new Shoot(shooter),
            new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.GROUND_PICKUP)),
            startAutoSequence(trajectory),
            new Shoot(shooter));
    }

    public Command ampSideAmpNote() {
        return commandBuilder(amp_side_amp.get(0), amp_side_amp.get(1));
    }

    public Command ampSideMiddleNote() {
        return commandBuilder(amp_side_middle.get(0), amp_side_middle.get(1));
    }

    public Command ampSideSourceNote() {
        return commandBuilder(amp_side_source.get(0), amp_side_source.get(1));
    }

    public Command ampSideFarLeft() {
        return commandBuilder(far_amp_side_left.get(0), far_amp_side_left.get(1));
    }

    public Command ampSideFarMiddleLeft() {
        return commandBuilder(far_amp_middle_left.get(0), far_amp_middle_left.get(1));
    }

    public Command ampSideFarMiddle() {
        return commandBuilder(far_amp_middle.get(0), far_amp_middle.get(1));
    }

    public Command middleSideMiddleNote() {
        return commandBuilder(note_speaker_3.get(0), note_speaker_3.get(1));
    }

    public Command middleSideAmpNote() {
        return commandBuilder(note_speaker_3.get(2), note_speaker_3.get(3));
    }

    public Command middleSideSourceNote() {
        return commandBuilder(note_speaker_3.get(4), note_speaker_3.get(5));
    }

    public Command sourceSideSourceNote() {
        return commandBuilder(source_side_source.get(0), source_side_source.get(1));
    }

    public Command sourceSideMiddleNote() {
        return commandBuilder(source_side_middle.get(0), source_side_middle.get(1));
    }

    public Command sourceSideAmpNote() {
        return commandBuilder(source_side_amp.get(0), source_side_amp.get(1));
    }

    public Command sourceSideFarRight() {
        return commandBuilder(far_source_right.get(0), far_source_right.get(1));
    }

    public Command sourceSideFarMiddleRight() {
        return commandBuilder(far_source_middle_right.get(0), far_source_middle_right.get(1));
    }

    public Command sourceFarMiddle() {
        return commandBuilder(source_far_middle.get(0), source_far_middle.get(1));
    }


    public Command fourNoteAmp() {
        return commandBuilder(Choreo.getTrajectory("four_note_w_amp"));
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
        return commandBuilder(middle_far_amp.get(0), middle_far_amp.get(1));
    }

    public Command middleFarMiddleAmp() {
        return commandBuilder(middle_far_middle_amp.get(0), middle_far_middle_amp.get(1));
    }

    public Command middleFarMiddle() {
        return commandBuilder(middle_far_middle.get(0), middle_far_middle.get(1));
    }

    public Command middleFarMiddleSource() {
        return commandBuilder(middle_far_middle_source.get(0), middle_far_middle_source.get(1));
    }

    public Command middleFarSource() {
        return commandBuilder(middle_far_source.get(0),  middle_far_source.get(1));
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
        Shooter shooter;
        private SendableChooser<Integer> startingPosition = new SendableChooser<>();
        private SendableChooser<Boolean> ampNote = new SendableChooser<>();
        private SendableChooser<Boolean> middleNote = new SendableChooser<>();
        private SendableChooser<Boolean> sourceNote = new SendableChooser<>();
        private SendableChooser<Boolean> ampFar = new SendableChooser<>();
        private SendableChooser<Boolean> ampMiddleFar = new SendableChooser<>();
        private SendableChooser<Boolean> farMiddle = new SendableChooser<>();
        private SendableChooser<Boolean> sourceFar = new SendableChooser<>();
        private SendableChooser<Boolean> sourceMiddleFar = new SendableChooser<>();

        public AutoSelector(DrivetrainBase drivetrainBase, Shooter shooter, VisionSubsystem visionSubsystem, VisionSubsystem visionSubsystemNote) {

            autoCommandFactory = new AutoCommandFactory(drivetrainBase, shooter, visionSubsystem, visionSubsystemNote);
            this.shooter = shooter;

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
            fullRoutine.add(new Shoot(shooter));
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
                if (middleNote.getSelected()) {
                    fullRoutine.add(autoCommandFactory.middleSideMiddleNote());
                }
                if (ampNote.getSelected()) {
                    fullRoutine.add(autoCommandFactory.middleSideAmpNote());
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
