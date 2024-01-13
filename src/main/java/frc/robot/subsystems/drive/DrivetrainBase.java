package frc.robot.subsystems.drive;

//import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.RobotState;
//import frc.robot.constants.ShuffleboardConstants;
//import frc.utils.subsystemUtils.StormSubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Toggles;

public abstract class DrivetrainBase extends SubsystemBase {

    public double m_maxVelocityMetersPerSecond;
    public double m_maxAngularVelocityRadiansPerSecond;
    protected double m_driveSpeedScale = 0;
    private final SlewRateLimiter speedScaleLimiter = new SlewRateLimiter(0.7);

    protected ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
//    protected final ShuffleboardTab tab;
    DrivetrainBase() {
        setDriveSpeedScale(Drive.driveSpeedScale);
        speedScaleLimiter.reset(m_driveSpeedScale);
//        tab = ShuffleboardConstants.getInstance().drivetrainTab;
    }

    protected void setMaxVelocities(double maxVelocityMetersPerSecond, double maxAngularVelocityRadiansPerSecond) {
        this.m_maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
        this.m_maxAngularVelocityRadiansPerSecond = maxAngularVelocityRadiansPerSecond;
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        // Scale incoming speeds
        ChassisSpeeds s = new ChassisSpeeds(
                speedScaleLimiter.calculate(m_driveSpeedScale) * speeds.vxMetersPerSecond,
                speedScaleLimiter.calculate(m_driveSpeedScale) * speeds.vyMetersPerSecond,
                speedScaleLimiter.calculate(m_driveSpeedScale) * speeds.omegaRadiansPerSecond);

        if (fieldRelative) {
            // TODO - use actual rotation!
            Rotation2d rotation = new Rotation2d();
//            var rotation = (usePoseEstimator)?
//                    RobotState.getInstance().getCurrentPose().getRotation() :
//                    RobotState.getInstance().getCurrentGyroData();
//            // if we are on the red team make sure field relative is the other way
//            if (RobotState.getInstance().getCurrentAlliance() == DriverStation.Alliance.Red) {
//                rotation = rotation.plus(new Rotation2d(Math.PI));
////                System.out.println(rotation);
//            }
            m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(s, rotation);
        } else {
            m_chassisSpeeds = s;
        }
    }

    public void percentOutDrive(ChassisSpeeds speeds, boolean fieldRelative) {
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

    public SwerveDriveKinematics getSwerveDriveKinematics() {return new SwerveDriveKinematics();}

    public SwerveModulePosition[] getSwerveModulePositions() {return new SwerveModulePosition[4];}
//    public void goToPPTrajectoryState(PathPlannerTrajectory.PathPlannerState goalState) {}
//    public boolean atReferenceState() {return true;}
//    public void updateOdometryData() {}
}