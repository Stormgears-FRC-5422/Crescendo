package frc.robot.subsystems.drive;


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.Drive;
import frc.robot.RobotState;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.encoders.CANCoderSwerve;
import swervelib.encoders.CanAndCoderSwerve;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.math.SwerveMath;
import swervelib.motors.SwerveMotor;
import swervelib.parser.SwerveDriveConfiguration;
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
            swerveDrive.resetOdometry(RobotState.getInstance().getStartPose());
        }

    @Override
    public void periodic() {
        swerveDrive.drive(m_chassisSpeeds);
        swerveDrive.updateOdometry();
        RobotState.getInstance().addPose(swerveDrive.getPose());
        RobotState.getInstance().setGyroData(swerveDrive.getYaw());
    }
}
