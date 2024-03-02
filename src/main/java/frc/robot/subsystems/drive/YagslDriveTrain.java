package frc.robot.subsystems.drive;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
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
import swervelib.motors.SwerveMotor;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;

import frc.robot.Constants.Swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class YagslDriveTrain extends DrivetrainBase {
    private final SwerveDrive swerveDrive;
    protected boolean m_localFieldRelative;
    private final StructPublisher<Pose2d> publisher;

    double maxVelocityMetersPerSecond = Constants.SparkMax.FreeSpeedRPM / 60.0 *
        Drive.driveReduction * Drive.wheelDiameter * Math.PI;
    double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond /
        Math.hypot(Drive.drivetrainTrackwidthMeters / 2.0, Drive.drivetrainWheelbaseMeters / 2.0);

    SwerveModule[] swerveModule;

    SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, null, null, // Use default config
            (state) -> Logger.recordOutput("SysIdTestState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (voltage) -> setVoltage(voltage.in(Volts)),
            null, // No log consumer, since data is recorded by AdvantageKit
            this
        )
    );


    YagslDriveTrain() throws IOException {

        super.setMaxVelocities(maxVelocityMetersPerSecond, maxAngularVelocityRadiansPerSecond);
        File directory = new File(Filesystem.getDeployDirectory(), Swerve.configDirectory);
        swerveDrive = new SwerveParser(directory).createSwerveDrive(m_maxVelocityMetersPerSecond);
        swerveDrive.setHeadingCorrection(false);
        swerveModule = swerveDrive.getModules();

        // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        // per https://www.chiefdelphi.com/t/yet-another-generic-swerve-library-yagsl-beta/425148/1280
//        if (SwerveDriveTelemetry.isSimulation) {
        for (SwerveModule m : swerveDrive.getModules()) {
            m.getConfiguration().useCosineCompensator = false;
        }
//        }

        // Before we know anything else, just assume forward is 0.
        swerveDrive.setGyro(new Rotation3d(0, 0, 0));

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

        Logger.recordOutput("Position", swerveDrive.getPose());
        Logger.recordOutput("Velocity", swerveDrive.getRobotVelocity());
        Logger.recordOutput("Volts", getVoltage());

    }


// The internal YAGSL implementation of zeroGyro
//    /**
//     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
//     */
//    public void zeroGyro() {
//        // Resets the real gyro or the angle accumulator, depending on whether the robot is being
//        // simulated
//        if (SwerveDriveTelemetry.isSimulation) {
//            simIMU.setAngle(0);
//        } else {
//            setGyroOffset(imu.getRawRotation3d());
//        }
//        imuReadingCache.update();
//        swerveController.lastAngleScalar = 0;
//        lastHeadingRadians = 0;
//        resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
//    }

    public void setVoltage(double v) {
        for (SwerveModule module : swerveModule) {
            module.getDriveMotor().setVoltage(v);
        }
    }

    public double getVoltage() {
        return swerveModule[0].getDriveMotor().getVoltage();
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

    public Command getQuasForwardCommand() {
        return Commands.sequence(new InstantCommand(this::zeroWheels), sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
    }

    public Command getQuasBackwardCommand() {
        return Commands.sequence(new InstantCommand(this::zeroWheels), sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));

    }

    public Command getDynamicForwardCommand() {
        return Commands.sequence(new InstantCommand(this::zeroWheels), sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
    }

    public Command getDynamicBackwardCommand() {
        return Commands.sequence(new InstantCommand(this::zeroWheels), sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    @Override
    public void zeroWheels() {
        for (SwerveModule sm : swerveModule) {
            sm.setAngle(0);
        }
    }

    @Override
    public void periodic() {
//        System.out.println("Gyro Roationt: " + swerveDrive.getYaw());
        publisher.set(swerveDrive.getPose());
        m_state.setPose(swerveDrive.getPose());

        if (m_localFieldRelative) {
            swerveDrive.driveFieldOriented(m_chassisSpeeds);
        } else {
            swerveDrive.drive(m_chassisSpeeds);
        }

//        m_state.setGyroData(swerveDrive.getYaw());
//        m_state.setPose(getPose());
    }


}
