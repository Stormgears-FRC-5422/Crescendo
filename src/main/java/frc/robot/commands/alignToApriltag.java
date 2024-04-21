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
import frc.utils.vision.LimelightHelpers;





public class alignToApriltag extends Command{


    double[] robotPose; //= LimelightHelpers.getBotPose_TargetSpace("");
    //horiz offset
    double movement = 0;
    double rotation = 0;

    private final PIDController translationController = new PIDController(0.5, 0, 0);

    private final PIDController rotationController = new PIDController(0.08, 0, 0.001);
    DrivetrainBase drivetrain;
    StormXboxController controller;

    NetworkTable table;
    VisionSubsystem visionSubsystem;


    public alignToApriltag(DrivetrainBase drivetrain, VisionSubsystem visionSubsystem) {
        translationController.setSetpoint(0.0);
        rotationController.setSetpoint(0.0);
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        addRequirements(drivetrain, visionSubsystem);
    }
    @Override
    public void initialize() {
        movement = 0;
        rotation = 0;
        translationController.setSetpoint(0.0);
        rotationController.setSetpoint(0.0);
    }
    @Override
    public void execute() {
        if (LimelightHelpers.getTV("")) {
            robotPose = LimelightHelpers.getBotPose_TargetSpace("");
            movement = robotPose[0];
            rotation = LimelightHelpers.getTX("");
            movement = translationController.calculate(movement);
            rotation = rotationController.calculate(rotation);
//            System.out.println("Note detected");
        }else{
            movement = 0;
            rotation = 0;
        }
        System.out.println("Movement: " + movement);
        System.out.println("ROT: " + rotation);
        ChassisSpeeds speeds = new ChassisSpeeds(0, movement + 0.25, rotation);
        drivetrain.drive(speeds, false, 1);
        Logger.recordOutput("movement" + movement);
        Logger.recordOutput("rotation" + rotation);
    }
    @Override
    public boolean isFinished() {
//        return RobotState.getInstance().getShooterState() == Shooter.ShooterState.STAGED_FOR_SHOOTING;
        //return (((tx > -0.25) && (tx < 0.25)) && ((ry > -2 && ry < 2)));
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("movement: " + movement);
        System.out.println("rotation: " + rotation);
        ChassisSpeeds speeds;
        speeds = new ChassisSpeeds(0, 0, 0);
        drivetrain.percentOutputDrive(speeds, false);
        System.out.println("Alignment To Apriltag Finished: interrupted:" + interrupted);
    }




}
