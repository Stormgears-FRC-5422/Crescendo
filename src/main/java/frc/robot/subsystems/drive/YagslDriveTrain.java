package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;

import frc.robot.Constants.Swerve;

public class YagslDriveTrain extends DrivetrainBase {

    private final SwerveDrive swerveDrive;

    YagslDriveTrain() throws IOException {
        File directory = new File(Filesystem.getDeployDirectory(),Swerve.configDirectory);

        swerveDrive = new SwerveParser(directory).createSwerveDrive(m_maxVelocityMetersPerSecond);
        swerveDrive.setHeadingCorrection(false);
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    }

    @Override
    public void periodic() {
        swerveDrive.drive(m_chassisSpeeds);
    }
}



