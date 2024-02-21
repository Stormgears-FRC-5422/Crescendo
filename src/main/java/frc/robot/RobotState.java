package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter;

public class RobotState extends SubsystemBase {
    public static enum StateAlliance {
        RED, BLUE, MISSING
    }
    private static RobotState m_instance;
    private StateAlliance m_alliance = StateAlliance.MISSING;
    private Pose2d currentPose = new Pose2d();
    private Shooter.ShooterState shooterState = Shooter.ShooterState.IDLE;
    private Pose2d visionPose = new Pose2d();

    public static RobotState getInstance() {
        if (m_instance != null) return m_instance;

        m_instance = new RobotState();
        return m_instance;
    }

    public void setAlliance(StateAlliance alliance) {
        m_alliance = alliance;
    }

    public StateAlliance getAlliance() {
        return m_alliance;
    }

    public boolean isAllianceBlue() {
        return m_alliance == StateAlliance.BLUE;
    }
    public boolean isAllianceRed() {
        return m_alliance == StateAlliance.RED;
    }
    public boolean isAllianceMissing() {
        return m_alliance == StateAlliance.MISSING;
    }

    public void setHeading(Rotation2d angle) {
        currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d(angle.getRadians()));
    }

    public Rotation2d getHeading() { return currentPose.getRotation(); }

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


}
