package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.vision.LimelightHelpers;

import java.util.Optional;

import static edu.wpi.first.math.util.Units.degreesToRadians;


public class RobotState extends SubsystemBase {
    public enum StateAlliance {
        RED, BLUE, MISSING
    }

    public enum StatePeriod {
        NONE, DISABLED, AUTONOMOUS, TELEOP, TEST
    }

    private static RobotState m_instance;
    private StateAlliance m_alliance = StateAlliance.MISSING;
    private Pose2d currentPose = new Pose2d();
    private Shooter.ShooterState shooterState = Shooter.ShooterState.IDLE;
    private Climber.ClimberState climberState = Climber.ClimberState.IDLE_BRAKE;
    //private Pose2d visionPose = new Pose2d();

    private StatePeriod m_period = StatePeriod.NONE;
    private boolean m_didAuto = false;
    private boolean m_didTeleop = false;
    private int count = 0;
    private boolean isPoseValid = false;
    private boolean climberIsHome = false;
    private boolean climberHasBeenHomed = false;
    private boolean isAtInit = false;
    boolean m_isUpperSensorTriggered;
    private boolean isNoteDetected;

    public static RobotState getInstance() {
        if (m_instance != null) return m_instance;

        m_instance = new RobotState();
        return m_instance;
    }

    public void setUpperSensorTriggered(boolean triggered) {
        m_isUpperSensorTriggered = triggered;
    }

    public boolean isUpperSensorTriggered() {
        return m_isUpperSensorTriggered;
    }

    public void setAlliance(StateAlliance alliance) {
        if (m_alliance != alliance) {
            System.out.println("**********");
            System.out.println("Alliance is now reported as " + alliance + ", was " + m_alliance);
            System.out.println("**********");
        }
        m_alliance = alliance;
    }

    public void setPeriod(StatePeriod period) {
        m_period = period;

        switch (period) {
            case AUTONOMOUS -> m_didAuto = true;
            case TELEOP -> m_didTeleop = true;
        }
    }

    public StatePeriod getPeriod() {
        return m_period;
    }

    public boolean getDidAuto() {
        return m_didAuto;
    }

    public boolean getDidTeleop() {
        return m_didTeleop;
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

    public Rotation2d getHeading() {
        return currentPose.getRotation();
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

    public Climber.ClimberState getClimberState() {
        return climberState;
    }

    public void setClimberState(Climber.ClimberState s) {
        climberState = s;
    }

    public void setClimberIsHome(boolean isHome) {
        climberIsHome = isHome;
    }

    public boolean isClimberHome() {
        return climberIsHome;
    }

    public void setClimberHasBeenHomed(boolean hasBeenHomed) {
        climberHasBeenHomed = hasBeenHomed;
    }

    public boolean climberHasBeenHomed() {
        if (Constants.Toggles.outReach) {
            return true;
        } else {
            return climberHasBeenHomed;
        }
    }

    public void setClimberIsAtInit(boolean atInit) {
        isAtInit = atInit;
    }

    public boolean isClimberAtInit() {
        return isAtInit;
    }

    public boolean isVisionPoseValid() {
        return LimelightHelpers.getTV(Constants.Vision.tagLimelight);
    }

/*    public void setVisionPose(Pose2d pose,boolean valid) {
        if (pose != null) {
            visionPose = pose;
        }
        isPoseValid = valid;
    }*/

    public Pose2d getVisionPose() {
//        Optional<LimelightHelpers.LimelightTarget_Fiducial> visionResult = vision.getLatestFiducialsTarget();
//        return toPose2D(visionResult.map(limelightTarget_fiducial -> limelightTarget_fiducial.botpose_wpiblue).orElse(null));
        return LimelightHelpers.getBotPose2d_wpiBlue(Constants.Vision.tagLimelight);
    }


    public void setIsNoteDetected(boolean detected) {
        isNoteDetected = detected;
    }

    public boolean getIsNoteDetected() {
        return isNoteDetected;
    }

    private static Pose2d toPose2D(double[] inData) {
        if (inData == null || inData.length < 6) {
            //System.err.println("Bad LL 2D Pose Data!");
            return null;
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }

}
