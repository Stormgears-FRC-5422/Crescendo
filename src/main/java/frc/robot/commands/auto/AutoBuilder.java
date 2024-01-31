package frc.robot.commands.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.CrescendoField;

import java.util.Objects;
import java.util.Optional;


public class AutoBuilder extends Command {

    DrivetrainBase drivetrain;
    ChoreoTrajectory traj = Choreo.getTrajectory("Trajectory");
    PIDController thetaController;

    public AutoBuilder(DrivetrainBase drivetrainBase) {
        this.drivetrain = drivetrainBase;
        thetaController = new PIDController(0, 0, 0);
//        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Command buildAuto(Alliance alliance) {
        boolean reflectField = alliance == Alliance.Red;

        Pose2d initialPose = CrescendoField.remapPose(traj.getInitialPose(), alliance);
        drivetrain.resetOdometry(initialPose);

        System.out.println("Starting command on " + (reflectField ? "Red" : "Blue") + " alliance.");
        System.out.println("Starting pose = " + initialPose);

        return Choreo.choreoSwerveCommand(
                traj,
                drivetrain::getPose,
                new PIDController(0, 0.0, 0.0),
                new PIDController(0, 0.0, 0.0),
                thetaController,
                (ChassisSpeeds speeds) -> drivetrain.drive(speeds,false, 1.0),
                () -> reflectField,
                drivetrain
        );

    }
}
