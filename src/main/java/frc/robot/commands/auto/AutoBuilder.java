package frc.robot.commands.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.Consumer;


public class AutoBuilder extends Command {

    DrivetrainBase drivetrainBase;
    ChoreoTrajectory traj = Choreo.getTrajectory("Trajectory");

    PIDController thetaController;

    public AutoBuilder(DrivetrainBase drivetrainBase) {
        this.drivetrainBase = drivetrainBase;
        thetaController = new PIDController(1, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Command buildAuto() {
        Command swerveCommand = Choreo.choreoSwerveCommand(
                traj,
                RobotState.getInstance()::getPose,
                new PIDController(1, 0.0, 0.0),
                new PIDController(1, 0.0, 0.0),
                thetaController,
                (ChassisSpeeds speeds) -> drivetrainBase.percentOutputDrive(
                        speeds,
                        false),
                () -> true,
                drivetrainBase
        );
        return swerveCommand;
    }
}
