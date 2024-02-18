package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter;

public class RobotState extends SubsystemBase {
    private static RobotState m_instance;
    private Alliance m_alliance = Alliance.Blue;
    private Rotation2d currentGyroData = new Rotation2d();
    private Pose2d currentPose = new Pose2d();
    private Shooter.ShooterState shooterState;
    private Field2d field2d;
    private Pose2d visionPose = new Pose2d();
    private Pose2d autoInitPose = new Pose2d();

    public static RobotState getInstance() {
        if (m_instance != null) return m_instance;

        m_instance = new RobotState();
        return m_instance;
    }

    public void setAlliance(Alliance alliance) {
        System.out.println("setAlliance to " + (alliance == Alliance.Blue ? "Blue" : "Red"));
        m_alliance = alliance;
    }

    public boolean isAllianceBlue() {
        return m_alliance == Alliance.Blue;
    }

    public void setGyroData(Rotation2d angle) {
        currentGyroData = angle;
    }

    public Rotation2d getCurrentGyroData() {
        return currentGyroData;
    }

    public void setPose(Pose2d pose) {
        // Make a copy, not a reference to the same object!
        currentPose = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(pose.getRotation().getRadians()));
    }

    public Pose2d getPose() {
        return currentPose;
    }

    public Shooter.ShooterState getShooterState() {
        return shooterState;
    }

    public void setShooterState(Shooter.ShooterState s) {
        shooterState = s;
    }


    public void setVisionPose(Pose2d pose) {
        if (pose != null) {
            visionPose = pose;
        }
    }

    public Pose2d getVisionPose() {
        return visionPose;
    }

    public void setAutoInitPose(Pose2d pose){
        autoInitPose = pose;
    }

    public Pose2d getAutoInitPose(){
        return  autoInitPose;
    }



}
