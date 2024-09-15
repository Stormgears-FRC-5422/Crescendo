package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.ctrGenerated.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.ctrGenerated.Telemetry;
import frc.robot.subsystems.drive.ctrGenerated.TunerConstants;
import frc.utils.LoggerWrapper;

public class CTRDrivetrain extends DrivetrainBase {
    RobotState robotState;
    final CommandSwerveDrivetrain drivetrain;
    final SwerveRequest.ApplyChassisSpeeds drive;
    final SwerveRequest.FieldCentric driveFieldOriented;
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    Telemetry logger = new Telemetry(MaxSpeed);
    SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    protected boolean m_localFieldRelative;


    public CTRDrivetrain() {
        robotState = RobotState.getInstance();

        drivetrain = TunerConstants.DriveTrain;

        drive = new SwerveRequest.ApplyChassisSpeeds()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
        driveFieldOriented = new SwerveRequest.FieldCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        setMaxVelocities(MaxSpeed, MaxAngularRate);
        drivetrain.registerTelemetry(logger::telemeterize);



        swerveDrivePoseEstimator = drivetrain.getPoseEstimator();
        swerveDrivePoseEstimator.update(drivetrain.getPigeon2().getRotation2d(), drivetrain.getModulePositions());

    }

    @Override
    public void resetOrientation() {
        drivetrain.getPigeon2().reset();
    }

    public Pose2d getPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void declarePoseIsNow(Pose2d newPose) {
        // These *MUST* be done in this order or odometry will be wrong.

        drivetrain.getPigeon2().setYaw(newPose.getRotation().getDegrees());
        resetPose(newPose);
        m_state.setPose(newPose);
    }

    public void resetPose(Pose2d pose) {
        swerveDrivePoseEstimator.resetPosition(
            drivetrain.getPigeon2().getRotation2d(),
            drivetrain.getModulePositions(), pose);
    }

    @Override
    public void drive(ChassisSpeeds speeds, boolean fieldRelative, double speedScale) {
        // Use the calculation from Drive base class, but don't let it do relative calculations
        // TODO - this is a bit hokey. We should have an explicit way to defer this to the drive subclass
        // without lying to the base class. That could cause other problems.
        super.drive(speeds, fieldRelative, speedScale);
        LoggerWrapper.recordOutput("Drive/DesiredSpeeds", speeds);
        m_localFieldRelative = fieldRelative;
//        m_localFieldRelative = false;
    }

    @Override
    public void periodic() {

        swerveDrivePoseEstimator.update(drivetrain.getPigeon2().getRotation2d(),
            drivetrain.getModulePositions());


//        if (m_localFieldRelative) {
//            swerveDrive.driveFieldOriented(m_chassisSpeeds);
//        } else {
//            swerveDrive.drive(m_chassisSpeeds);
//        }

        // ignoring field relative this should just be false.
//        System.out.println(m_chassisSpeeds.vxMetersPerSecond + " " + m_chassisSpeeds.vyMetersPerSecond + " " + m_chassisSpeeds.omegaRadiansPerSecond);
        if (m_localFieldRelative) {
            drivetrain.setControl(driveFieldOriented.withVelocityX(m_chassisSpeeds.vxMetersPerSecond)
                .withVelocityY(m_chassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(m_chassisSpeeds.omegaRadiansPerSecond));
        } else {
            drivetrain.setControl(drive.withSpeeds(m_chassisSpeeds));
        }
    }
}
