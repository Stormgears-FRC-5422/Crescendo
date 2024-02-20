package frc.robot.subsystems.drive;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.RobotState;
import frc.utils.LoggerWrapper;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;

import frc.robot.Constants.Swerve;

public class YagslDriveTrain extends DrivetrainBase {
    private final SwerveDrive swerveDrive;
    protected boolean m_localFieldRelative;
    private final StructPublisher<Pose2d> publisher;

    double maxVelocityMetersPerSecond = Constants.SparkMax.FreeSpeedRPM / 60.0 *
        Drive.driveReduction * Drive.wheelDiameter * Math.PI;
    double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond /
        Math.hypot(Drive.drivetrainTrackwidthMeters / 2.0, Drive.drivetrainWheelbaseMeters / 2.0);

    YagslDriveTrain() throws IOException {
        super.setMaxVelocities(maxVelocityMetersPerSecond, maxAngularVelocityRadiansPerSecond);
        File directory = new File(Filesystem.getDeployDirectory(), Swerve.configDirectory);
        swerveDrive = new SwerveParser(directory).createSwerveDrive(m_maxVelocityMetersPerSecond);
        swerveDrive.setHeadingCorrection(false);

        // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        // per https://www.chiefdelphi.com/t/yet-another-generic-swerve-library-yagsl-beta/425148/1280
        if (SwerveDriveTelemetry.isSimulation) {
            for (SwerveModule m: swerveDrive.getModules()) {
                m.getConfiguration().useCosineCompensator = false;
            }
        }

        swerveDrive.setGyroOffset(new Rotation3d(0,0,(Math.toRadians(Constants.NavX.navXOffsetDegrees)+RobotState.getInstance().getAutoInitPose().getRotation().getDegrees())));
        swerveDrive.setGyro(new Rotation3d(0, 0, Math.toRadians(Constants.NavX.navXOffsetDegrees)));

        SwerveDriveTelemetry.verbosity = switch (Swerve.verbosity.toLowerCase()) {
            case "high" -> SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
            case "medium" -> SwerveDriveTelemetry.TelemetryVerbosity.LOW;
            case "low" -> SwerveDriveTelemetry.TelemetryVerbosity.MACHINE;
            case "none" -> SwerveDriveTelemetry.TelemetryVerbosity.NONE;
            default -> SwerveDriveTelemetry.TelemetryVerbosity.NONE;
        };

        tab.addNumber("PoseX", () -> getPose().getX());
        tab.addNumber("ChassisSpeed", () -> m_chassisSpeeds.vxMetersPerSecond);
        tab.addNumber("YagslChassisiSpeeds", () -> getCurrentChassisSpeeds().vxMetersPerSecond);
        tab.addNumber("Vision X pose", () -> RobotState.getInstance().getVisionPose().getX());
        tab.addNumber("Vision Y Pose", () -> RobotState.getInstance().getVisionPose().getY());
        tab.addNumber("VIsion Rotation Pose", () -> RobotState.getInstance().getVisionPose().getRotation().getDegrees());

        //        needs to get moved just for testing
//      coold not figure out how to add pose through logger--- this allows us to use 3d field!!!
        publisher = NetworkTableInstance.getDefault()
            .getStructTopic("MyPose", Pose2d.struct).publish();
        Logger.recordOutput("MyPose", swerveDrive.getPose());



//        System.out.println("Current module positions:");
//        System.out.println(Arrays.toString(swerveDrive.getModulePositions()));
//        System.out.println(swerveDrive.getModules()[0].);
    }

    public void resetOdometry(Pose2d pose) {
        System.out.println("resetting pose to = " + pose);
        swerveDrive.resetOdometry(pose);
    }

    public void resetGyro() {
        Pose2d pose = swerveDrive.getPose();
        swerveDrive.zeroGyro();
        swerveDrive.resetOdometry(new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(0)));
    }

    @Override
    public void setGyroOffset() {
        swerveDrive.setGyroOffset(new Rotation3d(0,0,
            (Math.toRadians(Constants.NavX.navXOffsetDegrees+RobotState.getInstance().getAutoInitPose().getRotation().getDegrees()))));
    }

    @Override
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    @Override
    public void drive(ChassisSpeeds speeds, boolean fieldRelative, double speedScale) {
        // Use the calculation from Drive base class, but don't let it do relative calculations
        // TODO - this is a bit hokey. We should have an explicit way to defer this to the drive subclass
        // without lying to the base class. That could cause other problems.
        super.drive(speeds, false, speedScale);
        LoggerWrapper.recordOutput("Drive/DesiredSpeeds", speeds);
        m_localFieldRelative = fieldRelative;
    }

    @Override
    public void periodic() {

        publisher.set(swerveDrive.getPose());
//        System.out.println("Field Relative: " + m_localFieldRelative);
        if (m_localFieldRelative) {
//            System.out.println("yaw: " + swerveDrive.getYaw());
            swerveDrive.driveFieldOriented(m_chassisSpeeds);
        } else {
            swerveDrive.drive(m_chassisSpeeds);
        }

        // We can't use the navX directly, so we need to update the angle here
//        m_state.setGyroData(swerveDrive.getOdometryHeading());
        m_state.setGyroData(swerveDrive.getYaw());
        m_state.setPose(getPose());
    }
}
