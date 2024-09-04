package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.ctrGenerated.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.ctrGenerated.Telemetry;
import frc.robot.subsystems.drive.ctrGenerated.TunerConstants;
import frc.utils.LoggerWrapper;

public class CTRDrivetrain extends DrivetrainBase {
    RobotState robotState;
    final CommandSwerveDrivetrain drivetrain;
    final SwerveRequest.ApplyChassisSpeeds drive;
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    Telemetry logger = new Telemetry(MaxSpeed);

    protected boolean m_localFieldRelative;


    public CTRDrivetrain() {
        robotState = RobotState.getInstance();

        drivetrain = TunerConstants.DriveTrain;
        drive = new SwerveRequest.ApplyChassisSpeeds()
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        setMaxVelocities(MaxSpeed, MaxAngularRate);
        drivetrain.registerTelemetry(logger::telemeterize);

    }

    @Override
    public void drive(ChassisSpeeds speeds, boolean fieldRelative, double speedScale) {
        // Use the calculation from Drive base class, but don't let it do relative calculations
        // TODO - this is a bit hokey. We should have an explicit way to defer this to the drive subclass
        // without lying to the base class. That could cause other problems.
        super.drive(speeds, false, speedScale);
        LoggerWrapper.recordOutput("Drive/DesiredSpeeds", speeds);
//        m_localFieldRelative = fieldRelative;
        m_localFieldRelative = false;
    }

    @Override
    public void periodic() {
//        if (m_localFieldRelative) {
//            swerveDrive.driveFieldOriented(m_chassisSpeeds);
//        } else {
//            swerveDrive.drive(m_chassisSpeeds);
//        }

        // ignoring field relative this should just be false.
//        System.out.println(m_chassisSpeeds.vxMetersPerSecond + " " + m_chassisSpeeds.vyMetersPerSecond + " " + m_chassisSpeeds.omegaRadiansPerSecond);
        drivetrain.setControl(drive.withSpeeds(m_chassisSpeeds));
    }
}
