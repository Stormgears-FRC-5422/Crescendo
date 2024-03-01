package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Drive;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;


public class MecanumDrivetrain extends DrivetrainBase{
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;

    private SysIdRoutine sysIdRoutine;
    SysIdRoutineLog log = new SysIdRoutineLog("log");
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));


    private final MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(

            // Front left
            new Translation2d(Drive.drivetrainWheelbaseMeters / 2.0, Drive.drivetrainTrackwidthMeters / 2.0),
            // Front right
            new Translation2d(Drive.drivetrainWheelbaseMeters / 2.0, -Drive.drivetrainTrackwidthMeters / 2.0),
            // Back left
            new Translation2d(-Drive.drivetrainWheelbaseMeters / 2.0, Drive.drivetrainTrackwidthMeters / 2.0),
            // Back right
            new Translation2d(-Drive.drivetrainWheelbaseMeters / 2.0, -Drive.drivetrainTrackwidthMeters / 2.0)
    );

    //private final MecanumDriveOdometry m_odometry;

    private final WPI_TalonSRX m_frontLeftTalon;
    private final WPI_TalonSRX m_frontRightTalon;
    private final WPI_TalonSRX m_backLeftTalon;
    private final WPI_TalonSRX m_backRightTalon;

    public MecanumDrivetrain() {


        System.out.println("Creating Mecanum Drive");
        // TODO use StormTalon with voltage clamp
        m_frontLeftTalon = new WPI_TalonSRX(Drive.frontLeftDriveID);
        m_frontRightTalon = new WPI_TalonSRX(Drive.frontRightDriveID);
        m_backLeftTalon = new WPI_TalonSRX(Drive.backLeftDriveID);
        m_backRightTalon = new WPI_TalonSRX(Drive.backRightDriveID);


        m_frontLeftTalon.setInverted(false);
        m_backLeftTalon.setInverted(false);
        m_frontRightTalon.setInverted(true);
        m_backRightTalon.setInverted(true);




        double maxVelocityMetersPerSecond = 2 * Math.PI * Drive.wheelRadiusMeters * Drive.wheelMaxRPM / 60.0;
        double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond /
                Math.hypot(Drive.drivetrainTrackwidthMeters / 2.0, Drive.drivetrainWheelbaseMeters / 2.0);

        super.setMaxVelocities(maxVelocityMetersPerSecond, maxAngularVelocityRadiansPerSecond);
    }

    public void voltageDrive(MecanumDriveWheelSpeeds wheelSpeeds) {
        m_frontLeftTalon.setVoltage(MAX_VOLTAGE * wheelSpeeds.frontLeftMetersPerSecond / m_maxVelocityMetersPerSecond);
        m_frontRightTalon.setVoltage(MAX_VOLTAGE * wheelSpeeds.frontRightMetersPerSecond / m_maxVelocityMetersPerSecond);
        m_backLeftTalon.setVoltage(MAX_VOLTAGE * wheelSpeeds.rearLeftMetersPerSecond / m_maxVelocityMetersPerSecond);
        m_backRightTalon.setVoltage(MAX_VOLTAGE * wheelSpeeds.rearRightMetersPerSecond / m_maxVelocityMetersPerSecond);
    }

    @Override
    public void periodic() {
        MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(m_chassisSpeeds);
        wheelSpeeds.desaturate(this.m_maxVelocityMetersPerSecond);

        voltageDrive(wheelSpeeds);


    }

}
