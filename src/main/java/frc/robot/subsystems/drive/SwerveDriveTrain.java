package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.swervedrivespecialties.swervelib.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.utils.LoggerWrapper;

import static java.lang.Math.PI;

public class SwerveDriveTrain extends DrivetrainBase {
    public static final double m_maxMotorVoltage = Drive.maxMotorVoltage;
    CANcoder[] m_encoders = new CANcoder[4];
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

    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    SuppliedValueWidget<Double> CANabs;
    SuppliedValueWidget<Double> ABS;

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

        ShuffleboardLayout frontLeftModuleLayout = getShuffleboardLayout("Front Left Module", 2, 4, 0, 0);
        ShuffleboardLayout frontRightModuleLayout = getShuffleboardLayout("Front Right Module", 2, 4, 2, 0);
        ShuffleboardLayout backLeftModuleLayout = getShuffleboardLayout("Back Left Module", 2, 4, 4, 0);
        ShuffleboardLayout backRightModuleLayout = getShuffleboardLayout("Back Right Module", 2, 4, 6, 0);

        m_frontLeftModule = buildSwerveModule(frontLeftModuleLayout, Drive.frontLeftDriveID, Drive.frontLeftSteerID, Drive.frontLeftEncoderID, Drive.frontLeftOffsetDegrees);
        m_frontRightModule = buildSwerveModule(frontRightModuleLayout, Drive.frontRightDriveID, Drive.frontRightSteerID, Drive.frontRightEncoderID, Drive.frontRightOffsetDegrees);
        m_backLeftModule = buildSwerveModule(backLeftModuleLayout, Drive.backLeftDriveID, Drive.backLeftSteerID, Drive.backLeftEncoderID, Drive.backLeftOffsetDegrees);
        m_backRightModule = buildSwerveModule(backRightModuleLayout, Drive.backRightDriveID, Drive.backRightSteerID, Drive.backRightEncoderID, Drive.backRightOffsetDegrees);

        setMotorInvertedState(m_frontLeftModule.getDriveMotor(), false);
        setMotorInvertedState(m_frontRightModule.getDriveMotor(), false);
        setMotorInvertedState(m_backLeftModule.getDriveMotor(), false);
        setMotorInvertedState(m_backRightModule.getDriveMotor(), false);
        buildDriveShuffleBoard(m_frontLeftModule, m_encoders[0]);
        buildDriveShuffleBoard(m_frontRightModule, m_encoders[1]);
        buildDriveShuffleBoard(m_backLeftModule, m_encoders[2]);
        buildDriveShuffleBoard(m_backRightModule, m_encoders[3]);

    }

    @Override
    public void periodic() {
        states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, m_maxVelocityMetersPerSecond);

        setSwerveModuleVoltageAndSteerAngle(m_frontLeftModule, states[0], m_encoders[0]);
        setSwerveModuleVoltageAndSteerAngle(m_frontRightModule, states[1], m_encoders[1]);
        setSwerveModuleVoltageAndSteerAngle(m_backLeftModule, states[2], m_encoders[2]);
        setSwerveModuleVoltageAndSteerAngle(m_backRightModule, states[3], m_encoders[3]);
    }

    public void buildDriveShuffleBoard(SwerveModule swerveModule, CANcoder can) {
        int deviceID = ((CANSparkMax) swerveModule.getDriveMotor()).getDeviceId();
        tab.addNumber("CANAbsAngle_" + deviceID, () -> Math.toDegrees(can.getPosition().getValueAsDouble()));
        tab.addNumber("AbsAngle" + deviceID, () -> Math.toDegrees(swerveModule.getSteerEncoder().getAbsoluteAngle()));

    }

    private void setSwerveModuleVoltageAndSteerAngle(SwerveModule swerveModule, SwerveModuleState moduleState, CANcoder can) {
        int deviceID = ((CANSparkMax) swerveModule.getDriveMotor()).getDeviceId();
        swerveModule.set(m_maxMotorVoltage * moduleState.speedMetersPerSecond / m_maxVelocityMetersPerSecond, moduleState.angle.getRadians());
        LoggerWrapper.recordOutput("Drive/DeviceId" + deviceID, deviceID);
        LoggerWrapper.recordOutput("Drive/Distance" + deviceID, swerveModule.getDriveDistance());
        LoggerWrapper.recordOutput("Drive/SteerInverted" + deviceID, swerveModule.getSteerMotor().getInverted());
        LoggerWrapper.recordOutput("Drive/SteerAngle" + deviceID, swerveModule.getSteerAngle());
        LoggerWrapper.recordOutput("Drive/SteerEncoderAbsoluteAngle" + deviceID, swerveModule.getSteerEncoder().getAbsoluteAngle());
        LoggerWrapper.recordOutput("Drive/SteerEncoderAbsoluteAngleCAN" + deviceID, can.getPosition().getValue());

        SmartDashboard.putNumber("CANAbsAngle_" + deviceID, Math.toDegrees(can.getPosition().getValue()));
        SmartDashboard.putNumber("AbsAngle" + deviceID, Math.toDegrees(swerveModule.getSteerEncoder().getAbsoluteAngle()));
    }


    private void setMotorInvertedState(MotorController motorController, boolean invertedState) {
        if (motorController instanceof CANSparkMax) {
            ((CANSparkMax) motorController).setInverted(invertedState);
        }
    }

    private SwerveModule buildSwerveModule(ShuffleboardLayout moduleLayout, int driveId, int steerId, int encoderId, double offset) {
        return new MkSwerveModuleBuilder()
            .withLayout(moduleLayout)
            .withGearRatio(SdsModuleConfigurations.MK4_L2)
            .withDriveMotor(MotorType.NEO, driveId)
            .withSteerMotor(MotorType.NEO, steerId)
            .withSteerEncoderPort(encoderId)
            .withSteerOffset(-Math.toRadians(offset))
            .build();
    }

    private ShuffleboardLayout getShuffleboardLayout(String title, int width, int height, int colIndex, int rowIndex) {
        return tab.getLayout(title, BuiltInLayouts.kList)
            .withSize(width, height)
            .withPosition(colIndex, rowIndex);
    }

    void checkStatus(StatusCode statusCode, String operation) {
        if (statusCode.isWarning()) {
            System.out.printf("Warning on operation %s : %s", operation, statusCode.getName());
        } else if (statusCode.isError()) {
            System.out.printf("Error on operation %s : %s", operation, statusCode.getName());
        } // else OK - do nothing
    }

    private void initEncoders() {

        System.out.println("Initializing Encoders");
        try (
            CANcoder fl = new CANcoder(Drive.frontLeftEncoderID);
            CANcoder fr = new CANcoder(Drive.frontRightEncoderID);
            CANcoder bl = new CANcoder(Drive.backLeftEncoderID);
            CANcoder br = new CANcoder(Drive.backRightEncoderID);
        ) {
            m_encoders[0] = fl;
            m_encoders[1] = fr;
            m_encoders[2] = bl;
            m_encoders[3] = br;

            // from CTRE Github Phoenix6-Examples/java/FusedCANcoder/src/main/java/frc/robot
            // TODO check these configs wrt swerve modules
            CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
            cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
            cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

            // TODO - change this to match the proper units. For now assuming Radians
            double offsetScale = 2 * PI / Drive.SteerEncoderTicksPerRotation;

//            cc_cfg.MagnetSensor.MagnetOffset = Drive.frontLeftOffsetDegrees * offsetScale;
//            checkStatus(fl.getConfigurator().apply(cc_cfg), "set FL encoder");
//
//            cc_cfg.MagnetSensor.MagnetOffset = Drive.frontRightOffsetDegrees * offsetScale;
//            checkStatus(fr.getConfigurator().apply(cc_cfg), "set FR encoder");
//
//            cc_cfg.MagnetSensor.MagnetOffset = Drive.backLeftOffsetDegrees * offsetScale;
//            checkStatus(bl.getConfigurator().apply(cc_cfg), "set BL encoder");
//
//            cc_cfg.MagnetSensor.MagnetOffset = Drive.backRightOffsetDegrees * offsetScale;
//            checkStatus(br.getConfigurator().apply(cc_cfg), "set BR encoder");
        } catch (Exception e) {
            System.out.printf("Error on initEncoders. %s", e.getMessage());
            e.printStackTrace();
        }
    }
}

