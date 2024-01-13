package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.Drive.*;
import static frc.robot.Constants.Drive.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_OFFSET;

public class SwerveDriveTrain extends SubsystemBase {

    public double m_maxVelocityMetersPerSecond;

    public double m_maxAngularVelocityRadiansPerSecond;

    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);


    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);


    private final ShuffleboardTab tab = Shuffleboard.getTab("DriveTrain");

    private XboxController xboxController = new XboxController(0);

    public static final double MAX_VOLTAGE = 12.0;
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0)
    );

    private final SwerveModule
            m_frontLeftModule, m_frontRightModule,
            m_backLeftModule, m_backRightModule;

    public SwerveDrivetrain() {
        Mk4iSwerveModuleHelper.GearRatio gearRatio = Mk4iSwerveModuleHelper.GearRatio.L2;
        ModuleConfiguration moduleConfiguration = SdsModuleConfigurations.MK4_L2;

//        divide by 60 for rps instead of rpm
        double maxVelocityMetersPerSecond = kNeoFreeSpeedRPM / 60.0 *
                moduleConfiguration.getDriveReduction() *
                moduleConfiguration.getWheelDiameter() * Math.PI;
        double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond /
                Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
        ShuffleboardLayout frontLeftModuleLayout = tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0);
        ShuffleboardLayout frontRightModuleLayout = tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(2, 0);
        ShuffleboardLayout backLeftModuleLayout = tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(4, 0);
        ShuffleboardLayout backRightModuleLayout = tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(6, 0);

        m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
                frontLeftModuleLayout,
                gearRatio,
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET
        );

        m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
                frontRightModuleLayout,
                gearRatio,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
                backLeftModuleLayout,
                gearRatio,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET
        );

        m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
                backRightModuleLayout,
                gearRatio,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET
        );

        ((CANSparkMax) m_frontLeftModule.getDriveMotor()).setInverted(false);
        ((CANSparkMax) m_frontRightModule.getDriveMotor()).setInverted(false);
        ((CANSparkMax) m_backLeftModule.getDriveMotor()).setInverted(false);
        ((CANSparkMax) m_backRightModule.getDriveMotor()).setInverted(false);

        ((CANSparkMax) m_frontLeftModule.getDriveMotor()).setInverted(false);

    }
    public void drive(ChassisSpeeds speeds) {
        ChassisSpeeds speed = new ChassisSpeeds(speeds.vxMetersPerSecond * m_maxVelocityMetersPerSecond,
                speeds.vyMetersPerSecond * m_maxVelocityMetersPerSecond,
                speeds.omegaRadiansPerSecond * m_maxAngularVelocityRadiansPerSecond);
        Rotation2d rotation = m_gyro.getRotation2d();
        m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speed, rotation);

    }


    @Override
    public void periodic() {
        drive(new ChassisSpeeds(xboxController.getLeftX(),
                xboxController.getLeftY(), xboxController.getRightX()));
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, m_maxVelocityMetersPerSecond);

        m_frontLeftModule.set(MAX_VOLTAGE * states[0].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[0].angle.getRadians());
        m_frontRightModule.set(MAX_VOLTAGE * states[1].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[1].angle.getRadians());
        m_backLeftModule.set(MAX_VOLTAGE * states[2].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[2].angle.getRadians());
        m_backRightModule.set(MAX_VOLTAGE * states[3].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[3].angle.getRadians());
    }
}
