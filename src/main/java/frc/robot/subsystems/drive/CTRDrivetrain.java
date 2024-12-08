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
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.ctrGenerated.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.ctrGenerated.Telemetry;
import frc.robot.subsystems.drive.ctrGenerated.TunerConstants;
import frc.utils.LoggerWrapper;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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
    DoubleArraySubscriber poseSub;

    int count = 0;

    public CTRDrivetrain() {
        setDriveFlip(false);
        setFieldRelativeOn(false);
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Pose");
        poseSub = table.getDoubleArrayTopic("robotPose").subscribe(new double[] {});

        drivetrain = TunerConstants.DriveTrain;

        drive = new SwerveRequest.ApplyChassisSpeeds()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
        driveFieldOriented = new SwerveRequest.FieldCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        setMaxVelocities(MaxSpeed, MaxAngularRate);
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    @Override
    public void resetOrientation() {
//        Pose2d oldPose = getPose();
//
//        // Note that this behavior defaults to blue if alliance is missing.
        Rotation2d newRotation = m_state.isAllianceRed()
            ? new Rotation2d(-1, 0)
            : new Rotation2d(1, 0);
//         Rotation2d newRotation = new Rotation2d(1,0);

        System.out.println("Reset orientation at degrees: " + newRotation.getDegrees());

//
//        drivetrain.resetPose(newRotation);

        drivetrain.getPigeon2().setYaw(newRotation.getDegrees());
//        drivetrain.resetPose(newRotation);
    }

    public void declarePoseIsNow(Pose2d newPose) {
        // These *MUST* be done in this order or odometry will be wrong.


        drivetrain.getPigeon2().setYaw(newPose.getRotation().getDegrees());
        resetPose(newPose);
        m_state.setPose(newPose);
    }

    public void resetPose(Pose2d pose) {
        drivetrain.getPoseEstimator().resetPosition(
            drivetrain.getPigeon2().getRotation2d(),
            drivetrain.getModulePositions(), pose);
    }

    @AutoLogOutput
    public Pose2d getPose() {
        return new Pose2d(poseSub.get()[0],
            poseSub.get()[1],
            new Rotation2d(Math.toRadians(poseSub.get()[2])));
//        return drivetrain.getPoseEstimator().getEstimatedPosition();
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

        drivetrain.periodic();
        Logger.recordOutput("Pigeon Yaw", drivetrain.getPigeon2().getAngle());

//        drivetrain.getPoseEstimator().update(drivetrain.getPigeon2().getRotation2d(),
//            drivetrain.getModulePositions());


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
            if (count++ % 5 == 0
                && m_state.getPeriod() == RobotState.StatePeriod.AUTONOMOUS) {
                System.out.println("in CTRDrive chassisSpeeds: " + m_chassisSpeeds);
            }

            drivetrain.setControl(drive.withSpeeds(m_chassisSpeeds));
        }
    }
}
