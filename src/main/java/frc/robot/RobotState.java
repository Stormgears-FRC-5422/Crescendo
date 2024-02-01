package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotState extends SubsystemBase {

    private static RobotState m_instance;
    private Alliance m_alliance = Alliance.Blue;

    private Rotation2d currentGyroData = new Rotation2d();

    private Pose2d currentPose = new Pose2d();


    private Field2d field2d;

    public static RobotState getInstance() {
        if (m_instance != null) return m_instance;

        m_instance = new RobotState();
        return m_instance;
    }

    public void setAlliance(Alliance alliance) {
        m_alliance = alliance;
    }

    public boolean isAllianceBlue() {
        return m_alliance == Alliance.Blue;
    }

    public void setGyroData(Rotation2d angle) {
        currentGyroData = angle;
    }

    public Rotation2d getCurrentGyroData() {
//        if (!Constants.Toggles.useNavX) {
//            //System.out.println("NOT using gyro. Can't get current gyro rotation!");
//            return new Rotation2d();
//        }
        return currentGyroData;
    }

    public void setPose(Pose2d pose) {
        currentPose = pose;
    }

    public Pose2d getPose() {
        return currentPose;
    }


}
