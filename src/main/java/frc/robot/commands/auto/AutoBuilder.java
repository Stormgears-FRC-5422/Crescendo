package frc.robot.commands.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DrivetrainBase;


public class AutoBuilder extends Command {

    DrivetrainBase drivetrain;
    ChoreoTrajectory traj = Choreo.getTrajectory("Trajectory");
    PIDController thetaController;

    public AutoBuilder(DrivetrainBase drivetrainBase) {
        this.drivetrain = drivetrainBase;
        thetaController = new PIDController(0, 0, 0);
//        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Command buildAuto() {
        drivetrain.resetOdometry(traj.getInitialPose());

        return Choreo.choreoSwerveCommand(
                traj,
                drivetrain::getPose,
                new PIDController(0, 0.0, 0.0),
                new PIDController(0, 0.0, 0.0),
                thetaController,
                (ChassisSpeeds speeds) -> drivetrain.drive(speeds,true, 1.0),
                () -> false
            ,
                drivetrain);

    }
}
