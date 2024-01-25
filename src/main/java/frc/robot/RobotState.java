package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotState extends SubsystemBase {

    private static RobotState m_instance;

    private Rotation2d currentGyroData = null;

    private Pose2d currentPose;
    private Pose2d startPose;

    public static RobotState getInstance() {
        if (m_instance != null) return m_instance;

        m_instance = new RobotState();
        return m_instance;
    }

    public void setGyroData(Rotation2d angle) {
        currentGyroData = angle;
    }


    public Rotation2d getCurrentGyroData() {
        if (!Constants.Toggles.useNavX) {
            System.out.println("NOT using gyro. Can't get current gyro rotation!");
            return new Rotation2d();
        }
        return currentGyroData;
    }

    public void addPose(Pose2d pose) {
        currentPose = pose;
    }

    public Pose2d getPose(){
        return currentPose;
    }

    public Pose2d getStartPose() {
        if (startPose == null) {
//            System.out.println("Starting position was not set! Which is fine...");
            return new Pose2d();
        }
        return startPose;
    }

    public void setStartPose(Pose2d pose) {
        startPose = pose;
    }

}
