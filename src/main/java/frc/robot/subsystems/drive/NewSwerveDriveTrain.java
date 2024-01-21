package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;

public class NewSwerveDriveTrain extends DrivetrainBase {

    private final File directory = new File(Filesystem.getDeployDirectory(),"swerve");

    private final SwerveDrive swerveDrive;

    NewSwerveDriveTrain() throws IOException {

        swerveDrive = new SwerveParser(directory).createSwerveDrive(m_maxVelocityMetersPerSecond);
        swerveDrive.setHeadingCorrection(false);
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

    }




    @Override
    public void periodic() {
        swerveDrive.drive(m_chassisSpeeds);
    }
}



