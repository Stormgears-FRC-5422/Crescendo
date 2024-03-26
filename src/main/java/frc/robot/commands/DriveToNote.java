package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.joysticks.StormXboxController;

import static java.lang.Math.abs;

public class DriveToNote extends Command {

    private static int TARGET = -21;
    private final PIDController translationController = new PIDController(0.15, 0, 0);
    private final PIDController rotationController = new PIDController(0.03, 0, 0);
    DrivetrainBase drivetrain;
    StormXboxController controller;
    double tx;
    double ty;
    NetworkTable table;
    VisionSubsystem visionSubsystem;


    public DriveToNote(DrivetrainBase drivetrain, VisionSubsystem visionSubsystem) {
        translationController.setSetpoint(0.0);
        rotationController.setSetpoint(0.0);
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        addRequirements(drivetrain, visionSubsystem);
    }


    @Override
    public void execute() {
        double movement = 0;
        double rotation = 0;
        if (visionSubsystem.getLatestDetectorTarget().isPresent()) {
            tx = visionSubsystem.getLatestDetectorTarget().get().tx;
            ty = visionSubsystem.getLatestDetectorTarget().get().ty;
            movement = translationController.calculate(TARGET - ty);
            rotation = rotationController.calculate(tx);
        }
        ChassisSpeeds speeds = new ChassisSpeeds(movement, 0, rotation);
        drivetrain.percentOutputDrive(speeds, false);
        }
    }


    @Override
    public boolean isFinished() {
        return RobotState.getInstance().getShooterState() == Shooter.ShooterState.STAGED_FOR_SHOOTING;
    }

    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds speeds;
        speeds = new ChassisSpeeds(0, 0, 0);
        drivetrain.percentOutputDrive(speeds, false);
        System.out.println("Drive To Note Finished");
    }
}
