package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import com.swervedrivespecialties.swervelib.MechanicalConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.MotorType;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import frc.robot.Constants;
import frc.robot.Constants.Drive;

import static java.lang.Math.PI;

public class SwerveDriveTrain extends DrivetrainBase {
    private final SwerveModule m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule;
    public static final double m_maxMotorVoltage = Drive.maxMotorVoltage;
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

    public SwerveDriveTrain() {
        initEncoders();

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
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, m_maxVelocityMetersPerSecond);

        m_frontLeftModule.set(m_maxMotorVoltage * states[0].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[0].angle.getRadians());
        m_frontRightModule.set(m_maxMotorVoltage * states[1].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[1].angle.getRadians());
        m_backLeftModule.set(m_maxMotorVoltage * states[2].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[2].angle.getRadians());
        m_backRightModule.set(m_maxMotorVoltage * states[3].speedMetersPerSecond / m_maxVelocityMetersPerSecond, states[3].angle.getRadians());
    }

    void checkStatus(StatusCode statusCode, String operation) {
        if (statusCode.isWarning()) {
            System.out.println("Warning on operation " + operation + ": " + statusCode.getName());
        } else if (statusCode.isError()) {
            System.out.println("Error on operation " + operation + ": " + statusCode.getName());
        } // else OK - do nothing
    }

    private void initEncoders() {
        try (
           CANcoder fl = new CANcoder(Drive.frontLeftEncoderID);
           CANcoder fr = new CANcoder(Drive.frontRightEncoderID);
           CANcoder bl = new CANcoder(Drive.backLeftEncoderID);
           CANcoder br = new CANcoder(Drive.backRightEncoderID);
        ) {
            // from CTRE Github Phoenix6-Examples/java/FusedCANcoder/src/main/java/frc/robot
            // TODO check these configs wrt swerve modules
            CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
            cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
            cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

            // TODO - change this to match the proper units. For now assuming Radians
            double offsetScale = 2 * PI / Drive.steerEncoderTicksPerRotation;

            cc_cfg.MagnetSensor.MagnetOffset = Drive.frontLeftOffsetTicks * offsetScale;
            checkStatus(fl.getConfigurator().apply(cc_cfg), "set FL encoder");

            cc_cfg.MagnetSensor.MagnetOffset = Drive.frontRightOffsetTicks * offsetScale;
            checkStatus(fr.getConfigurator().apply(cc_cfg), "set FR encoder");

            cc_cfg.MagnetSensor.MagnetOffset = Drive.backLeftOffsetTicks * offsetScale;
            checkStatus(bl.getConfigurator().apply(cc_cfg), "set BL encoder");

            cc_cfg.MagnetSensor.MagnetOffset = Drive.backRightOffsetTicks * offsetScale;
            checkStatus(br.getConfigurator().apply(cc_cfg), "set BR encoder");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}

