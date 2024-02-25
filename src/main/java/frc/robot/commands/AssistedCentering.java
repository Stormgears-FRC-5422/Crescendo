package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.joysticks.CrescendoJoystick;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AssistedCentering extends Command {
    double horizontalOffset;
    VisionSubsystem visionSubsystem;
    DrivetrainBase drivetrain;
    CrescendoJoystick joystick;
    DoubleSupplier WPI_X;
    DoubleSupplier WPI_OMEGA;
    double x;
    double omega;
    BooleanSupplier robotRelativeSupplier;
    boolean fieldRelative;



    public AssistedCentering(VisionSubsystem visionSubsystem, DrivetrainBase drivetrain, CrescendoJoystick joystick){
        this.visionSubsystem = visionSubsystem;
        this.drivetrain = drivetrain;
        this.joystick = joystick;
        addRequirements(visionSubsystem,drivetrain);
        WPI_X = joystick::getWpiX;
        WPI_OMEGA = joystick::getOmegaSpeed;
        robotRelativeSupplier = joystick::getRobotRelative;
    }

    public boolean isLeft(){
        return (horizontalOffset <  -0.1);
    }
    public boolean isRight(){
        return (horizontalOffset > 0.1);
    }


    private void centerRobot(){
        if(isLeft()) {
            ChassisSpeeds driveSpeed = new  ChassisSpeeds(x,0.1,omega);
            drivetrain.drive(driveSpeed,true);
        }else if(isRight()){
            ChassisSpeeds driveSpeed = new  ChassisSpeeds(x,-0.1,omega);
            drivetrain.drive(driveSpeed,true);
        } else {
            ChassisSpeeds driveSpeed = new  ChassisSpeeds(x,0,omega);
            drivetrain.drive(driveSpeed,true);
        }
    }


    public void periodic(){
        x = WPI_X.getAsDouble();
        omega = WPI_OMEGA.getAsDouble();
        fieldRelative = !robotRelativeSupplier.getAsBoolean();
        centerRobot();
        horizontalOffset = visionSubsystem.getHorizOffsetFromAprilTag();
    }
}
