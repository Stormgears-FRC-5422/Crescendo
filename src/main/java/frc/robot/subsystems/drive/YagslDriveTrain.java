package frc.robot.subsystems.drive;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;

import frc.robot.Constants.Swerve;

public class YagslDriveTrain extends DrivetrainBase {
    private final SwerveDrive swerveDrive;
    protected boolean m_localFieldRelative;

    double maxVelocityMetersPerSecond = Constants.SparkMax.FreeSpeedRPM / 60.0 *
        Drive.driveReduction * Drive.wheelDiameter * Math.PI;
    double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond /
        Math.hypot(Drive.drivetrainTrackwidthMeters / 2.0, Drive.drivetrainWheelbaseMeters / 2.0);

    YagslDriveTrain() throws IOException {
        super.setMaxVelocities(maxVelocityMetersPerSecond, maxAngularVelocityRadiansPerSecond);
        File directory = new File(Filesystem.getDeployDirectory(), Swerve.configDirectory);
        swerveDrive = new SwerveParser(directory).createSwerveDrive(m_maxVelocityMetersPerSecond);
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setGyroOffset(swerveDrive.getGyroRotation3d());
        swerveDrive.setGyro(new Rotation3d(0, 0, 3.1415));

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
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    @Override
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    @Override
    public void drive(ChassisSpeeds speeds, boolean fieldRelative, double speedScale) {
        // Use the calculation from Drive base class, but don't let it do relative calculations
        // TODO - this is a bit hokey. We should have an explicit way to defer this to the drive subclass
        // without lying to the base class. That could cause other problems.
        super.drive(speeds, false, speedScale);
        m_localFieldRelative = fieldRelative;
    }

    @Override
    public void periodic() {
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
