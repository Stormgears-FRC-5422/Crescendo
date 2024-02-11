package frc.robot.subsystems.drive;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.Drive;

public class SwerveDiagnosticDriveTrain extends DrivetrainBase {
    CANSparkMax m_frontLeftDrive, m_frontRightDrive, m_backLeftDrive, m_backRightDrive;
    CANSparkMax m_frontLeftSteer, m_frontRightSteer, m_backLeftSteer, m_backRightSteer;
    CANSparkMax[] m_driveArray;
    CANSparkMax[] m_steerArray;

    public static final double m_maxMotorVoltage = Drive.maxMotorVoltage;

    public SwerveDiagnosticDriveTrain() {
        // These are convenient lies
        double maxVelocityMetersPerSecond = m_maxMotorVoltage;
        double maxAngularVelocityRadiansPerSecond = m_maxMotorVoltage;

        super.setMaxVelocities(maxVelocityMetersPerSecond*0.2, maxAngularVelocityRadiansPerSecond*0.2);

        m_frontLeftDrive = new CANSparkMax(Drive.frontLeftDriveID, MotorType.kBrushless);
        m_frontLeftSteer = new CANSparkMax(Drive.frontLeftSteerID, MotorType.kBrushless);
        m_frontRightDrive = new CANSparkMax(Drive.frontRightDriveID, MotorType.kBrushless);
        m_frontRightSteer = new CANSparkMax(Drive.frontRightSteerID, MotorType.kBrushless);

        m_backLeftDrive = new CANSparkMax(Drive.backLeftDriveID, MotorType.kBrushless);
        m_backLeftSteer = new CANSparkMax(Drive.backLeftSteerID, MotorType.kBrushless);
        m_backRightDrive = new CANSparkMax(Drive.backRightDriveID, MotorType.kBrushless);
        m_backRightSteer = new CANSparkMax(Drive.backRightSteerID, MotorType.kBrushless);

        m_driveArray = new CANSparkMax[]{m_frontLeftDrive, m_frontRightDrive,
                                       m_backLeftDrive, m_backRightDrive};
        m_steerArray = new CANSparkMax[]{m_frontLeftSteer, m_frontRightSteer,
                                       m_backLeftSteer, m_backRightSteer};

        // Initialize to known states
        for (CANSparkMax m : m_driveArray) {
            m.setInverted(true);
            m.setIdleMode(CANSparkBase.IdleMode.kCoast);
        }

        for (CANSparkMax m : m_steerArray) {
            m.setInverted(true);
            m.setIdleMode(CANSparkBase.IdleMode.kCoast);
        }
    }

    @Override
    public void periodic() {
        double driveSpeed = m_chassisSpeeds.vxMetersPerSecond;
        double steerSpeed = m_chassisSpeeds.omegaRadiansPerSecond;

        for (CANSparkMax m : m_driveArray) {
            m.set(driveSpeed);
        }

        for (CANSparkMax m : m_steerArray) {
            m.set(steerSpeed);
        }
    }

    public void resetOdometry(Pose2d pose) {
    }

    public void resetGyro(){}

}
