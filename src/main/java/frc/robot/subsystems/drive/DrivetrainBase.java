package frc.robot.subsystems.drive;

//import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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


    protected ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    //    protected final ShuffleboardTab tab;
    DrivetrainBase() {
        setDriveSpeedScale(Drive.driveSpeedScale);
        tab = ShuffleboardConstants.getInstance().drivetrainTab;
    }

    protected void setMaxVelocities(double maxVelocityMetersPerSecond, double maxAngularVelocityRadiansPerSecond) {
        this.m_maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
        this.m_maxAngularVelocityRadiansPerSecond = maxAngularVelocityRadiansPerSecond;
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
        if (fieldRelative) {
            Rotation2d rotation = RobotState.getInstance().getCurrentGyroData();
            m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation);
        }

        // TODO - work in the slew rate limiter. Not sure whether to apply before or after scale
        m_chassisSpeeds = scaleChassisSpeeds(m_chassisSpeeds, m_driveSpeedScale);
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

//    protected Rotation2d getGyroscopeRotation() {
//        return RobotState.getInstance().getCurrentGyroData();
//    }

    public ChassisSpeeds getCurrentChassisSpeeds() {
        return m_chassisSpeeds;
    }

    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return new SwerveDriveKinematics();
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[4];
    }
//    public void goToPPTrajectoryState(PathPlannerTrajectory.PathPlannerState goalState) {}
//    public boolean atReferenceState() {return true;}
//    public void updateOdometryData() {}


}

