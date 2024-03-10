package frc.robot.subsystems.drive;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.RobotState;
import frc.utils.LoggerWrapper;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;

import frc.robot.Constants.Swerve;

import static edu.wpi.first.units.Units.Volts;

public class YagslDriveTrain extends DrivetrainBase {
    private final SwerveDrive swerveDrive;
    protected boolean m_localFieldRelative;
    private final StructPublisher<Pose2d> publisher;

    double maxVelocityMetersPerSecond = Constants.SparkMax.FreeSpeedRPM / 60.0 *
        Drive.driveReduction * Drive.wheelDiameter * Math.PI;
    double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond /
        Math.hypot(Drive.drivetrainTrackwidthMeters / 2.0, Drive.drivetrainWheelbaseMeters / 2.0);

    SwerveModule[] swerveModules;

    YagslDriveTrain() throws IOException {
        super.setMaxVelocities(maxVelocityMetersPerSecond, maxAngularVelocityRadiansPerSecond);
        File directory = new File(Filesystem.getDeployDirectory(), Swerve.configDirectory);
        swerveDrive = new SwerveParser(directory).createSwerveDrive(m_maxVelocityMetersPerSecond);
        swerveDrive.setHeadingCorrection(false);
        swerveModules = swerveDrive.getModules();

        // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        // per https://www.chiefdelphi.com/t/yet-another-generic-swerve-library-yagsl-beta/425148/1280
        // There are other reasons to disable it (e.g. during PID tuning)
        if (SwerveDriveTelemetry.isSimulation || !Swerve.useCosineCompensation) {
            for (SwerveModule sm : swerveModules) {
                sm.getConfiguration().useCosineCompensator = false;
            }
        }

        if (Swerve.useVoltageCompensation) {
            for (SwerveModule sm : swerveModules) {
                sm.setDriveMotorVoltageCompensation(Swerve.compensatedVoltage);
            }
        }

        // Before we know anything else, just assume forward is 0.
        swerveDrive.setGyro(new Rotation3d(0, 0, 0));

        SwerveDriveTelemetry.verbosity = switch (Swerve.verbosity.toLowerCase()) {
            case "high" -> SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
            case "medium" -> SwerveDriveTelemetry.TelemetryVerbosity.LOW;
            case "low" -> SwerveDriveTelemetry.TelemetryVerbosity.MACHINE;
            case "none" -> SwerveDriveTelemetry.TelemetryVerbosity.NONE;
            default -> SwerveDriveTelemetry.TelemetryVerbosity.NONE;
        };

//        tab.addNumber("PoseX", () -> getPose().getX());
//        tab.addNumber("ChassisSpeed", () -> m_chassisSpeeds.vxMetersPerSecond);
//        tab.addNumber("YagslChassisiSpeeds", () -> getCurrentChassisSpeeds().vxMetersPerSecond);
//        tab.addNumber("Vision X pose", () -> RobotState.getInstance().getVisionPose().getX());
//        tab.addNumber("Vision Y Pose", () -> RobotState.getInstance().getVisionPose().getY());
//        tab.addNumber("VIsion Rotation Pose", () -> RobotState.getInstance().getVisionPose().getRotation().getDegrees());

        //        needs to get moved just for testing
//      coold not figure out how to add pose through logger--- this allows us to use 3d field!!!
        publisher = NetworkTableInstance.getDefault()
            .getStructTopic("MyPose", Pose2d.struct).publish();

    }

    @Override
    public void setHeadingCorrectionTrue() {
        swerveDrive.setHeadingCorrection(true);
    }

    @Override
    public void resetOrientation() {
        Pose2d oldPose = getPose();

        // Note that this behavior defaults to blue if alliance is missing.
        Rotation2d newRotation = m_state.isAllianceRed()
            ? new Rotation2d(-1, 0)
            : new Rotation2d(1, 0);

        Pose2d newPose = new Pose2d(oldPose.getX(), oldPose.getY(), newRotation);
        declarePoseIsNow(newPose);
    }

    public void declarePoseIsNow(Pose2d newPose) {
        // These *MUST* be done in this order or odometry will be wrong.
        setGyro(newPose.getRotation());
        resetOdometry(newPose);
        m_state.setPose(newPose);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
     * This is independent of odometry - you probably don't want to call this
     */
    private void setGyro(Rotation2d angle) {
        // TODO - for now, keep navXOffsetDegrees = 0. Need to better understand the interplay between
        // the physical offset details of the gyro and pose. Should be just an offset, but too many things
        // are connected together to be sure which are invariant.
        System.out.println("resetting gyro to rotation = " + angle.getDegrees() + " degrees");
        swerveDrive.setGyro(new Rotation3d(0, 0,
            angle.getRadians() + 0));
    }

    private void resetOdometry(Pose2d pose) {
        System.out.println("resetting odometry to pose = " + pose);
        swerveDrive.resetOdometry(pose);
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

    SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, null, null, // Use default config
            (state) -> Logger.recordOutput("SysIdTestState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (voltage) -> setSysIdVoltage(voltage.in(Volts)),
            null, // No log consumer, since data is recorded by AdvantageKit
            this
        )
    );

    public void setSysIdVoltage(double v) {
        for (SwerveModule sm : swerveModules) {
            sm.setAngle(0);
            sm.getDriveMotor().setVoltage(v);
        }
    }

    public double getSysIdVoltage() {
        return swerveModules[0].getDriveMotor().getVoltage();
    }

    public Command getQuasForwardCommand() {
        return Commands.sequence(
            new InstantCommand(this::zeroWheels),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
        );
    }

    @Override
    public Command getSysIdCommand() {
        return SwerveDriveTest.generateSysIdCommand(sysIdRoutine, 1, 2, 2);

    }

    public Command getQuasBackwardCommand() {
        return Commands.sequence(
            new InstantCommand(this::zeroWheels),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
        );
    }

    public Command getDynamicForwardCommand() {
        return Commands.sequence(
            new InstantCommand(this::zeroWheels),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
        );
    }

    public Command getDynamicBackwardCommand() {
        return Commands.sequence(
            new InstantCommand(this::zeroWheels),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)

        );
    }

    @Override
    public void zeroWheels() {
        for (SwerveModule sm : swerveModules) {
            sm.setAngle(0);
        }
    }

    @Override
    public void periodic() {
        publisher.set(swerveDrive.getPose());
        m_state.setPose(swerveDrive.getPose());

        if (m_localFieldRelative) {
            swerveDrive.driveFieldOriented(m_chassisSpeeds);
        } else {
            swerveDrive.drive(m_chassisSpeeds);
        }
        for (int i = 0; i < swerveModules.length; i++) {
            Logger.recordOutput("Position drive module " + i, swerveModules[i].getDriveMotor().getPosition());
            Logger.recordOutput("Velocity drive module " + i, swerveModules[i].getDriveMotor().getVelocity());
            Logger.recordOutput("Volts drive module " + i, swerveModules[i].getDriveMotor().getVoltage());
            Logger.recordOutput("Position angle module " + i, swerveModules[i].getAngleMotor().getPosition());
            Logger.recordOutput("Velocity angle module " + i, swerveModules[i].getAngleMotor().getVelocity());
            Logger.recordOutput("Volts angle module " + i, swerveModules[i].getAngleMotor().getVoltage());
        }

    }
}
