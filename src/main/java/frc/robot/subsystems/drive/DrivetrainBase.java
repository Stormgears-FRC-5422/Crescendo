package frc.robot.subsystems.drive;

//import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.RobotState;
import frc.robot.ShuffleboardConstants;

public abstract class DrivetrainBase extends SubsystemBase {

    public double m_maxVelocityMetersPerSecond = 1 ;
    public double m_maxAngularVelocityRadiansPerSecond = 1;
    protected double m_driveSpeedScale = 0;
    private final SlewRateLimiter speedScaleLimiter = new SlewRateLimiter(0.7);
    protected final ShuffleboardTab tab;

    protected final RobotState m_state;
    protected boolean m_fieldRelative = false;
    protected ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    //    protected final ShuffleboardTab tab;
    DrivetrainBase() {
        setDriveSpeedScale(Drive.driveSpeedScale);
        tab = ShuffleboardConstants.getInstance().drivetrainTab;
        m_state = RobotState.getInstance();
    }

    protected void setMaxVelocities(double maxVelocityMetersPerSecond, double maxAngularVelocityRadiansPerSecond) {
        m_maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
        m_maxAngularVelocityRadiansPerSecond = maxAngularVelocityRadiansPerSecond;
        System.out.println("MaxDRIVEVal: " + m_maxVelocityMetersPerSecond);
        System.out.println("MaxANgleVal: " + m_maxAngularVelocityRadiansPerSecond);
    }

    // Be careful scaling ChassisSpeeds. Need to scale X and Y the same or your robot will move in the wrong direction!
    public ChassisSpeeds scaleChassisSpeeds(ChassisSpeeds speeds, double scale) {
        return new ChassisSpeeds(scale * speeds.vxMetersPerSecond,
                                 scale * speeds.vyMetersPerSecond,
                                 scale * speeds.omegaRadiansPerSecond);
    }

    /**
     * Command the robot to drive.
     * This method expects real speeds in meters/second.
     * Speed may be limited by speedScale and / or slew rate limiter
     * @param speeds        Chassis speedsSwerveDriveConfiguration for the swerve, esp from joystick.
     * @param fieldRelative True for field relative driving
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        drive(speeds, fieldRelative, m_driveSpeedScale);
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative, double speedScale) {
        m_fieldRelative = fieldRelative;

        if (fieldRelative) {
            Rotation2d rotation = m_state.getHeading();
            m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation);
        } else {
            m_chassisSpeeds = speeds;
        }

        // TODO - work in the slew rate limiter. Apply before scale to preserve motion details
        m_chassisSpeeds = scaleChassisSpeeds(m_chassisSpeeds, speedScale);

    }

    /**
     * Command the robot to drive, especially from Joystick
     * This method expects units from -1 to 1, and then scales them to the max speeds
     * You should call setMaxVelocities() before calling this method
     * @param speeds        Chassis speeds, especially from joystick.
     * @param fieldRelative True for field relative driving
     */
    public void percentOutputDrive(ChassisSpeeds speeds, boolean fieldRelative) {
        drive(new ChassisSpeeds(speeds.vxMetersPerSecond * m_maxVelocityMetersPerSecond,
                        speeds.vyMetersPerSecond * m_maxVelocityMetersPerSecond,
                        speeds.omegaRadiansPerSecond * m_maxAngularVelocityRadiansPerSecond),
                fieldRelative);
    }

    public void setDriveSpeedScale(double scale) {
        m_driveSpeedScale = MathUtil.clamp(scale, 0, Drive.driveSpeedScale);
    }

    public void stopDrive() {
        drive(new ChassisSpeeds(0, 0, 0), false);
    }

    public ChassisSpeeds getCurrentChassisSpeeds() {
        return m_chassisSpeeds;
    }

    public Pose2d getPose(){
        return new Pose2d();
    }

    // Teach the drive that the current orientation is facing the opposite end of the field
    // this function is ideally alliance aware, and manages the pose and gyro as needed
    public void resetOrientation() {
    }

    public void declarePoseIsNow(Pose2d pose) {
    }

}

