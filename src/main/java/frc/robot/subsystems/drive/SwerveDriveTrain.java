package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.swervedrivespecialties.swervelib.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants;
import frc.robot.Constants.Drive;

public class SwerveDriveTrain extends DrivetrainBase {

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
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    public SwerveDriveTrain() {

        MechanicalConfiguration moduleConfiguration = SdsModuleConfigurations.MK4_L2;

//        divide by 60 for rps instead of rpm
        double maxVelocityMetersPerSecond = Constants.SparkMax.FreeSpeedRPM / 60.0 *
                moduleConfiguration.getDriveReduction() *
                moduleConfiguration.getWheelDiameter() * Math.PI;

        double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond /
                Math.hypot(Drive.drivetrainTrackwidthMeters / 2.0, Drive.drivetrainWheelbaseMeters / 2.0);

        super.setMaxVelocities(maxVelocityMetersPerSecond, maxAngularVelocityRadiansPerSecond);

        ShuffleboardLayout frontLeftModuleLayout = getShuffleboardLayout("Front Left Module", 2,4, 0,0);
        ShuffleboardLayout frontRightModuleLayout = getShuffleboardLayout("Front Right Module", 2,4, 2,0);
        ShuffleboardLayout backLeftModuleLayout = getShuffleboardLayout("Back Left Module", 2,4, 4,0);
        ShuffleboardLayout backRightModuleLayout = getShuffleboardLayout("Back Right Module", 2,4, 6,0);

        m_frontLeftModule = buildSwerveModule(frontLeftModuleLayout, Drive.frontLeftDriveID, Drive.frontLeftSteerID, Drive.frontLeftEncoderID, Drive.frontLeftOffsetTicks);
        m_frontRightModule = buildSwerveModule(frontRightModuleLayout, Drive.frontRightDriveID, Drive.frontRightSteerID, Drive.frontRightEncoderID, Drive.frontRightOffsetTicks);
        m_backLeftModule = buildSwerveModule(backLeftModuleLayout, Drive.backLeftDriveID, Drive.backLeftSteerID, Drive.backLeftEncoderID, Drive.backLeftOffsetTicks);
        m_backRightModule = buildSwerveModule(backRightModuleLayout, Drive.backRightDriveID, Drive.backRightSteerID, Drive.backRightEncoderID, Drive.backRightOffsetTicks);

        setMotorInvertedState(m_frontLeftModule.getDriveMotor(), false);
        setMotorInvertedState(m_frontRightModule.getDriveMotor(), false);
        setMotorInvertedState(m_backLeftModule.getDriveMotor(), false);
        setMotorInvertedState(m_backRightModule.getDriveMotor(), false);
    }

    @Override
    public void periodic() {

        states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, m_maxVelocityMetersPerSecond);

        setSwerveModuleVoltageAndSteerAngle(m_frontLeftModule, states[0]);
        setSwerveModuleVoltageAndSteerAngle(m_frontRightModule, states[1]);
        setSwerveModuleVoltageAndSteerAngle(m_backLeftModule, states[2]);
        setSwerveModuleVoltageAndSteerAngle(m_backRightModule, states[3]);
    }

    private void setSwerveModuleVoltageAndSteerAngle(SwerveModule swerveModule, SwerveModuleState moduleState) {
        swerveModule.set(MAX_VOLTAGE * moduleState.speedMetersPerSecond/m_maxVelocityMetersPerSecond, moduleState.angle.getRadians());
    }

    private void setMotorInvertedState(MotorController motorController, boolean invertedState) {
        if(motorController instanceof CANSparkMax) {
            ((CANSparkMax) motorController).setInverted(invertedState);
        }
    }

    private SwerveModule buildSwerveModule(ShuffleboardLayout moduleLayout, int driveId, int steerId, int encoderId, int offsetTicks) {
        return new MkSwerveModuleBuilder()
                .withLayout(moduleLayout)
                .withGearRatio(SdsModuleConfigurations.MK4_L2)
                .withDriveMotor(MotorType.NEO, driveId)
                .withSteerMotor(MotorType.NEO, steerId)
                .withSteerEncoderPort(encoderId)
                .withSteerOffset(offsetTicks)
                .build();
    }

    private ShuffleboardLayout getShuffleboardLayout(String title, int width, int height, int colIndex, int rowIndex) {
        return tab.getLayout(title, BuiltInLayouts.kList)
                .withSize(width, height)
                .withPosition(colIndex, rowIndex);
    }
}

