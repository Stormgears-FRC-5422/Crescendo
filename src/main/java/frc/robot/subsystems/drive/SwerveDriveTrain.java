package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import com.swervedrivespecialties.swervelib.MechanicalConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.MotorType;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.drive.DrivetrainBase;

public class SwerveDriveTrain extends DrivetrainBase {

    public double m_maxVelocityMetersPerSecond;

    public double m_maxAngularVelocityRadiansPerSecond;

    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);


    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);


    private final ShuffleboardTab tab = Shuffleboard.getTab("DriveTrain");


    public static final double MAX_VOLTAGE = 12.0;
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(Drive.drivetrainWheelbaseMeters / 2.0, Drive.drivetrainTrackwidthMeters / 2.0),
            // Front right
            new Translation2d(Drive.drivetrainWheelbaseMeters / 2.0, -Drive.drivetrainTrackwidthMeters / 2.0),
            // Back left
            new Translation2d(-Drive.drivetrainWheelbaseMeters / 2.0, Drive.drivetrainTrackwidthMeters / 2.0),
            // Back right
            new Translation2d(-Drive.drivetrainWheelbaseMeters / 2.0, -Drive.drivetrainTrackwidthMeters / 2.0)
    );

    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);


    private final SwerveModule
            m_frontLeftModule, m_frontRightModule,
            m_backLeftModule, m_backRightModule;

    public SwerveDriveTrain() {

        MechanicalConfiguration moduleConfiguration = SdsModuleConfigurations.MK4_L2;

//        divide by 60 for rps instead of rpm
        double maxVelocityMetersPerSecond = Constants.SparkMax.FreeSpeedRPM / 60.0 *
                moduleConfiguration.getDriveReduction() *
                moduleConfiguration.getWheelDiameter() * Math.PI;
        double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond /
                Math.hypot(Drive.drivetrainTrackwidthMeters / 2.0, Drive.drivetrainWheelbaseMeters / 2.0);

        super.setMaxVelocities(maxVelocityMetersPerSecond, maxAngularVelocityRadiansPerSecond);


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

        m_frontLeftModule = new MkSwerveModuleBuilder()
                .withLayout(frontLeftModuleLayout)
                .withGearRatio(SdsModuleConfigurations.MK4_L2)
                .withDriveMotor(MotorType.NEO, Drive.frontLeftDriveID)
                .withSteerMotor(MotorType.NEO, Drive.frontLeftSteerID)
                .withSteerEncoderPort(Drive.frontLeftEncoderID)
                .withSteerOffset(Drive.frontLeftOffsetTicks)
                .build();

        m_frontRightModule = new MkSwerveModuleBuilder()
                .withLayout(frontRightModuleLayout)
                .withGearRatio(SdsModuleConfigurations.MK4_L2)
                .withDriveMotor(MotorType.NEO, Drive.frontRightDriveID)
                .withSteerMotor(MotorType.NEO, Drive.frontRightSteerID)
                .withSteerEncoderPort(Drive.frontRightEncoderID)
                .withSteerOffset(Drive.frontRightOffsetTicks)
                .build();

        m_backLeftModule = new MkSwerveModuleBuilder()
                .withLayout(backLeftModuleLayout)
                .withGearRatio(SdsModuleConfigurations.MK4_L2)
                .withDriveMotor(MotorType.NEO, Drive.backLeftDriveID)
                .withSteerMotor(MotorType.NEO, Drive.backLeftSteerID)
                .withSteerEncoderPort(Drive.backLeftEncoderID)
                .withSteerOffset(Drive.backLeftOffsetTicks)
                .build();

        m_backRightModule = new MkSwerveModuleBuilder()
                .withLayout(backRightModuleLayout)
                .withGearRatio(SdsModuleConfigurations.MK4_L2)
                .withDriveMotor(MotorType.NEO, Drive.backRightDriveID)
                .withSteerMotor(MotorType.NEO, Drive.backRightSteerID)
                .withSteerEncoderPort(Drive.backRightEncoderID)
                .withSteerOffset(Drive.backRightOffsetTicks)
                .build();

        ((CANSparkMax) m_frontLeftModule.getDriveMotor()).setInverted(false);
        ((CANSparkMax) m_frontRightModule.getDriveMotor()).setInverted(false);
        ((CANSparkMax) m_backLeftModule.getDriveMotor()).setInverted(false);
        ((CANSparkMax) m_backRightModule.getDriveMotor()).setInverted(false);
    }



    @Override
    public void periodic() {

        states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, m_maxVelocityMetersPerSecond);

        System.out.println("FL: " + states[0].angle.getRotations());
        System.out.println("FR: " + states[1].angle.getRotations());
        System.out.println("BL: " + states[2].angle.getRotations());
        System.out.println("BR: " + states[3].angle.getRotations());

        m_frontLeftModule.set(MAX_VOLTAGE * states[0].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[0].angle.getRadians());
        m_frontRightModule.set(MAX_VOLTAGE * states[1].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[1].angle.getRadians());
        m_backLeftModule.set(MAX_VOLTAGE * states[2].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[2].angle.getRadians());
        m_backRightModule.set(MAX_VOLTAGE * states[3].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[3].angle.getRadians());
    }
}
