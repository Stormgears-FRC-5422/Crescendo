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

    public double m_maxVelocityMetersPerSecond;
    public double m_maxAngularVelocityRadiansPerSecond;
    protected double m_driveSpeedScale = 0;
    private final SlewRateLimiter speedScaleLimiter = new SlewRateLimiter(0.7);

    protected final ShuffleboardTab tab;


    protected ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    //    protected final ShuffleboardTab tab;
    DrivetrainBase() {

        setDriveSpeedScale(Drive.driveSpeedScale);
        speedScaleLimiter.reset(m_driveSpeedScale);
        tab = ShuffleboardConstants.getInstance().drivetrainTab;
    }


    protected void setMaxVelocities(double maxVelocityMetersPerSecond, double maxAngularVelocityRadiansPerSecond) {
        this.m_maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
        this.m_maxAngularVelocityRadiansPerSecond = maxAngularVelocityRadiansPerSecond;
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        ChassisSpeeds limitSpeed = new ChassisSpeeds(speeds.vxMetersPerSecond * m_maxVelocityMetersPerSecond,
                speeds.vyMetersPerSecond * m_maxVelocityMetersPerSecond,
                speeds.omegaRadiansPerSecond * m_maxAngularVelocityRadiansPerSecond);
        if (fieldRelative) {
            Rotation2d rotation = RobotState.getInstance().getCurrentGyroData();
            m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(limitSpeed, rotation);
        } else {
            m_chassisSpeeds = limitSpeed;
        }
    }

//    public void percentOutDrive(ChassisSpeeds speeds, boolean fieldRelative) {
//        drive(new ChassisSpeeds(speeds.vxMetersPerSecond * m_maxVelocityMetersPerSecond,
//                        speeds.vyMetersPerSecond * m_maxVelocityMetersPerSecond,
//                        speeds.omegaRadiansPerSecond * m_maxAngularVelocityRadiansPerSecond),
//                fieldRelative);
//    }

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

