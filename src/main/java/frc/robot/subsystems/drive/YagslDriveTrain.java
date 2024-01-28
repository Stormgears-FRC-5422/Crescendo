package frc.robot.subsystems.drive;


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
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
import frc.robot.Constants.Drive;

public class YagslDriveTrain extends DrivetrainBase {
    private final SwerveDrive swerveDrive;


    double maxVelocityMetersPerSecond = Constants.SparkMax.FreeSpeedRPM / 60.0 *
        Drive.driveReduction * Drive.wheelDiameter * Math.PI;
    double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond /
        Math.hypot(Drive.drivetrainTrackwidthMeters / 2.0, Drive.drivetrainWheelbaseMeters / 2.0);

    YagslDriveTrain() throws IOException {
        super.setMaxVelocities(maxVelocityMetersPerSecond, maxAngularVelocityRadiansPerSecond);
        File directory = new File(Filesystem.getDeployDirectory(), Swerve.configDirectory);
        swerveDrive = new SwerveParser(directory).createSwerveDrive(m_maxVelocityMetersPerSecond);
        swerveDrive.setHeadingCorrection(false);
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
//        swerveDrive.resetDriveEncoders();
        swerveDrive.resetOdometry(pose);
    }

    @Override
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    @Override
    public void periodic() {
//        if (fieldRelative) {
        swerveDrive.driveFieldOriented(m_chassisSpeeds);
//        System.out.println("Yaw: " + swerveDrive.getYaw());
//        } else {
//            swerveDrive.drive(m_chassisSpeeds);
//        }
        RobotState.getInstance().setGyroData(swerveDrive.getYaw());
        RobotState.getInstance().setPose(getPose());
    }
}
