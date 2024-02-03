package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import frc.robot.Constants;

public class MecanumDiagnosticDrivetrain extends DrivetrainBase {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;

    private final MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(

            // Front left
            new Translation2d(Constants.Drive.drivetrainWheelbaseMeters / 2.0, Constants.Drive.drivetrainTrackwidthMeters / 2.0),
            // Front right
            new Translation2d(Constants.Drive.drivetrainWheelbaseMeters / 2.0, -Constants.Drive.drivetrainTrackwidthMeters / 2.0),
            // Back left
            new Translation2d(-Constants.Drive.drivetrainWheelbaseMeters / 2.0, Constants.Drive.drivetrainTrackwidthMeters / 2.0),
            // Back right
            new Translation2d(-Constants.Drive.drivetrainWheelbaseMeters / 2.0, -Constants.Drive.drivetrainTrackwidthMeters / 2.0)
    );

    //private final MecanumDriveOdometry m_odometry;

    private final WPI_TalonSRX m_frontLeftTalon;
    private final WPI_TalonSRX m_frontRightTalon;
    private final WPI_TalonSRX m_backLeftTalon;
    private final WPI_TalonSRX m_backRightTalon;

    public MecanumDiagnosticDrivetrain() {

        System.out.println("Creating Mecanum Drive");
        // TODO use StormTalon with voltage clamp
        m_frontLeftTalon = new WPI_TalonSRX(Constants.Drive.frontLeftDriveID);
        m_frontRightTalon = new WPI_TalonSRX(Constants.Drive.frontRightDriveID);
        m_backLeftTalon = new WPI_TalonSRX(Constants.Drive.backLeftDriveID);
        m_backRightTalon = new WPI_TalonSRX(Constants.Drive.backRightDriveID);

        m_frontLeftTalon.setInverted(false);
        m_backLeftTalon.setInverted(false);
        m_frontRightTalon.setInverted(true);
        m_backRightTalon.setInverted(true);
    }

    @Override
    public void periodic() {
        System.out.println(m_chassisSpeeds.vxMetersPerSecond);

        m_frontLeftTalon.set(m_chassisSpeeds.vxMetersPerSecond);
        m_frontRightTalon.set(m_chassisSpeeds.vxMetersPerSecond);
        m_backLeftTalon.set(m_chassisSpeeds.vxMetersPerSecond);
        m_backRightTalon.set(m_chassisSpeeds.vxMetersPerSecond);

    }
}
