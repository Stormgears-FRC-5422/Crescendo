package frc.robot.subsystems.drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.utils.swerve.SwerveConstants;
import frc.utils.swerve.SwerveModule;
import org.littletonrobotics.junction.AutoLogOutput;

public class KrishDrive extends DrivetrainBase {

    RobotState robotState = RobotState.getInstance();

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(Constants.Drive.drivetrainWheelbaseMeters / 2.0, Constants.Drive.drivetrainTrackwidthMeters / 2.0),
            // Front right
            new Translation2d(Constants.Drive.drivetrainWheelbaseMeters / 2.0, -Constants.Drive.drivetrainTrackwidthMeters / 2.0),
            // Back right
            new Translation2d(-Constants.Drive.drivetrainWheelbaseMeters / 2.0, -Constants.Drive.drivetrainTrackwidthMeters / 2.0),
            // Back left
            new Translation2d(-Constants.Drive.drivetrainWheelbaseMeters / 2.0, Constants.Drive.drivetrainTrackwidthMeters / 2.0)
    );

    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;


    double maxVelocityMetersPerSecond = Constants.Drive.FreeSpeedRPM / 60.0 *
            Constants.Drive.driveReduction * Constants.Drive.wheelDiameter * Math.PI;
    double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond /
            Math.hypot(Constants.Drive.drivetrainTrackwidthMeters / 2.0, Constants.Drive.drivetrainWheelbaseMeters / 2.0);

    @AutoLogOutput
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    @AutoLogOutput
    SwerveModuleState[] realStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds());


    public KrishDrive() {


        m_frontLeftModule = new SwerveModule(1, new SwerveModule.SwerveModuleConstants(11, 10, Constants.Drive.frontLeftOffsetDegrees),
                SwerveConstants.mFrontLeftCancoder);
        m_frontRightModule = new SwerveModule(2, new SwerveModule.SwerveModuleConstants(21, 20, Constants.Drive.frontRightOffsetDegrees),
                SwerveConstants.mFrontRightCancoder);
        m_backRightModule = new SwerveModule(3, new SwerveModule.SwerveModuleConstants(31, 30, Constants.Drive.backRightOffsetDegrees),
                SwerveConstants.mBackRightCancoder);
        m_backLeftModule = new SwerveModule(4, new SwerveModule.SwerveModuleConstants(41, 40, Constants.Drive.backLeftOffsetDegrees),
                SwerveConstants.mBackLeftCancoder);
//        m_frontLeftModule = new SwerveModule(1, new SwerveModule.SwerveModuleConstants(11, 10, 0),
//            SwerveConstants.mFrontLeftCancoder);
//        m_frontRightModule = new SwerveModule(2, new SwerveModule.SwerveModuleConstants(21, 20, 0),
//            SwerveConstants.mFrontRightCancoder);
//        m_backRightModule = new SwerveModule(3, new SwerveModule.SwerveModuleConstants(31, 30, 0),
//            SwerveConstants.mBackRightCancoder);
//        m_backLeftModule = new SwerveModule(4, new SwerveModule.SwerveModuleConstants(41, 40, 0),
//            SwerveConstants.mBackLeftCancoder);

        setMaxVelocities(maxVelocityMetersPerSecond, maxAngularVelocityRadiansPerSecond);

//
//        SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics,
//            new Rotation2d(), realStates, RobotState.getInstance().getPose());
    }


    @Override
    public void periodic() {


        states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

//        System.out.println(states[0].angle.getDegrees());

        if (m_chassisSpeeds.vxMetersPerSecond != 0 || m_chassisSpeeds.vyMetersPerSecond != 0 || m_chassisSpeeds.omegaRadiansPerSecond != 0) {
//            System.out.println("omega " + m_chassisSpeeds.omegaRadiansPerSecond);
//            System.out.println("vy" + m_chassisSpeeds.vyMetersPerSecond);
//            System.out.println("vx" + m_chassisSpeeds.vxMetersPerSecond);
            m_frontLeftModule.setVelocity(states[0]);
            m_frontRightModule.setVelocity(states[1]);
            m_backRightModule.setVelocity(states[2]);
            m_backLeftModule.setVelocity(states[3]);
        }
        realStates = new SwerveModuleState[]{
                m_frontLeftModule.getState(),
                m_frontRightModule.getState(),
                m_backRightModule.getState(),
                m_backLeftModule.getState()
        };


    }
}
