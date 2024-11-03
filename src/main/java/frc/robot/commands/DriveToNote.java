package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.joysticks.StormXboxController;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;

import static java.lang.Math.abs;

public class DriveToNote extends Command {

    private static int TARGET =  -Constants.DriveToNote.targetOffset;
    private final PIDController translationController = new PIDController(Constants.DriveToNote.translationKp,
        Constants.DriveToNote.translationKi, Constants.DriveToNote.translationKd);
    private final PIDController rotationController = new PIDController(Constants.DriveToNote.rotationKp,
        Constants.DriveToNote.rotationKi, Constants.DriveToNote.rotationKd);

    DrivetrainBase drivetrain;
    StormXboxController controller;
    double tx;
    double ty;
    NetworkTable table;
    VisionSubsystem visionSubsystem;
    int count = 0;
    double movement = 0;
    double rotation = 0;



    public DriveToNote(DrivetrainBase drivetrain, VisionSubsystem visionSubsystem) {
        translationController.setSetpoint(0.0);
        rotationController.setSetpoint(0.0);
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        addRequirements(drivetrain, visionSubsystem);
    }

    @Override
    public void initialize() {
        count = 0;
        movement = 0;
        rotation = 0;
        translationController.setSetpoint(0.0);
        rotationController.setSetpoint(0.0);
    }

    @Override
    public void execute() {
        rotation = 0;
        if (visionSubsystem.getValid()) {
            tx = visionSubsystem.getTX();
            ty = visionSubsystem.getTX();
            movement = translationController.calculate(TARGET - ty);
            rotation = rotationController.calculate(tx);
            System.out.println("Note detected");
            count = 0;
        } else {
            count++;
        }
        System.out.println("Movement: " + movement);
        System.out.println("ROT: " + rotation);
        System.out.println("count: " + count);
        ChassisSpeeds speeds = new ChassisSpeeds(movement, 0, rotation);
        drivetrain.drive(speeds, false, 1);
        Logger.recordOutput("Tx" + tx);
        Logger.recordOutput("Ty" + ty);

    }


    @Override
    public boolean isFinished() {
       return RobotState.getInstance().isUpperSensorTriggered() || count >10;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Tx: " + tx);
        System.out.println("Ty: " + ty);
        ChassisSpeeds speeds;
        speeds = new ChassisSpeeds(0, 0, 0);
        drivetrain.percentOutputDrive(speeds, false);
        System.out.println("Drive To Note Finished: interrupted:" + interrupted);
    }
}
