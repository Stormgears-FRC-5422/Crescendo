package frc.robot.commands.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;

import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.CrescendoField;


public class AutoCommands extends Command {
    final RobotState robotState;
    DrivetrainBase drivetrain;

    public AutoCommands(DrivetrainBase drivetrainBase) {
        this.drivetrain = drivetrainBase;
        robotState = RobotState.getInstance();
    }

    public Command buildAuto(String trajectoryName) {
//        TODO need a better way of getting trajectory name
        ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryName);
        boolean openLoop = Swerve.openLoopAuto;
        boolean reflectField = !robotState.isAllianceBlue();
        Pose2d initialPose = CrescendoField.remapPose(trajectory.getInitialPose(), robotState.isAllianceBlue());

        System.out.println("Building auto command on " + (reflectField ? "Red" : "Blue") + " alliance");
        System.out.println("Starting pose = " + initialPose);
        System.out.println("Control is " + (openLoop ? "open loop" : "pid controlled"));

        // It looks like we don't want to reuse the controller objects
        // I don't see anything in the source that would reset their state when the command is created.
        // We don't want lingering integration values, for example
        // Use openLoop to disable PID entirely and just get raw motion
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

        // TODO - this way to set the initial pose is probably temporary. We could use something more sophisticated -
        // for example, based on vision detection of an April Tag to make sure we aren't in the entirely wrong place
        drivetrain.resetOdometry(initialPose);
        return Choreo.choreoSwerveCommand(
                trajectory,
                drivetrain::getPose,
                xController,
                yController,
                rotationController,
                (ChassisSpeeds speeds) -> drivetrain.drive(speeds,false, 1.0),
                () -> reflectField,
                drivetrain
        );

    }


    public Command test2m() {
        Command choreoTraj = buildAuto("simple_2m");
//        empty for now bc have not added shooter and intake
        return choreoTraj;
    }

    public Command fourNoteAmp() {
        Command choreoTraj = buildAuto("four_note_w_amp");
//        empty for now bc have not added shooter and intake

        return choreoTraj;
    }
}
