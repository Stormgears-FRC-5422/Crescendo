package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Drive;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveModule;

import static edu.wpi.first.units.Units.Volts;

public class SwerveDiagnosticDriveTrain extends DrivetrainBase {
    CANSparkMax m_frontLeftDrive, m_frontRightDrive, m_backLeftDrive, m_backRightDrive;
    CANSparkMax m_frontLeftSteer, m_frontRightSteer, m_backLeftSteer, m_backRightSteer;
    CANSparkMax[] m_driveArray;
    CANSparkMax[] m_steerArray;

    public static final double m_maxMotorVoltage = Drive.maxMotorVoltage;

    PowerDistribution powerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);


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

    public SwerveDiagnosticDriveTrain() {
        // These are convenient lies
        double maxVelocityMetersPerSecond = m_maxMotorVoltage;
        double maxAngularVelocityRadiansPerSecond = m_maxMotorVoltage;

        super.setMaxVelocities(maxVelocityMetersPerSecond * 0.2, maxAngularVelocityRadiansPerSecond * 0.2);

        m_frontLeftDrive = new CANSparkMax(Drive.frontLeftDriveID, MotorType.kBrushless);
        m_frontLeftSteer = new CANSparkMax(Drive.frontLeftSteerID, MotorType.kBrushless);
        m_frontRightDrive = new CANSparkMax(Drive.frontRightDriveID, MotorType.kBrushless);
        m_frontRightSteer = new CANSparkMax(Drive.frontRightSteerID, MotorType.kBrushless);

        m_backLeftDrive = new CANSparkMax(Drive.backLeftDriveID, MotorType.kBrushless);
        m_backLeftSteer = new CANSparkMax(Drive.backLeftSteerID, MotorType.kBrushless);
        m_backRightDrive = new CANSparkMax(Drive.backRightDriveID, MotorType.kBrushless);
        m_backRightSteer = new CANSparkMax(Drive.backRightSteerID, MotorType.kBrushless);

        m_driveArray = new CANSparkMax[]{m_frontLeftDrive, m_frontRightDrive,
            m_backLeftDrive, m_backRightDrive};
        m_steerArray = new CANSparkMax[]{m_frontLeftSteer, m_frontRightSteer,
            m_backLeftSteer, m_backRightSteer};

        // Initialize to known states
        for (CANSparkMax m : m_driveArray) {
            m.setInverted(!m.equals(m_frontLeftDrive) && !m.equals(m_backLeftDrive));
            m.setIdleMode(CANSparkBase.IdleMode.kCoast);
            m.getEncoder().setAverageDepth(8);
            m.getEncoder().setMeasurementPeriod(8);
            m.setSmartCurrentLimit(80);
        }

//        for (CANSparkMax m : m_driveA
//       rray) {
//
//            m.setInverted(true);
//            m.setIdleMode(CANSparkBase.IdleMode.kCoast);
//        }
//
        for (CANSparkMax m : m_steerArray) {
            m.setInverted(true);
            m.setIdleMode(CANSparkBase.IdleMode.kCoast);
        }


    }

    public void setSysIdVoltage(double v) {
        for (CANSparkMax m : m_driveArray) {
            m.setVoltage(v);
        }
//        System.out.println("Sysid VOlts: " + v);
        Logger.recordOutput("sysid volts", v);
    }

    public Command getQuasForwardCommand() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(5);
    }


    public Command getQuasBackwardCommand() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(5);
    }

    public Command getDynamicForwardCommand() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(1.5

        );
    }

    public Command getDynamicBackwardCommand() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(1.5);
    }


    @Override
    public void periodic() {
        double driveSpeed = m_chassisSpeeds.vxMetersPerSecond;
        double steerSpeed = m_chassisSpeeds.omegaRadiansPerSecond;
//
//        for (CANSparkMax m : m_driveArray) {
//            m.set(driveSpeed);
//        }
////
//        for (CANSparkMax m : m_steerArray) {
//            m.set(steerSpeed);
//        }

        Logger.recordOutput("Module Number 1 " , powerDistribution.getCurrent(13));
        Logger.recordOutput("Module Number 2 " , powerDistribution.getCurrent(8));
        Logger.recordOutput("Module Number 3 " , powerDistribution.getCurrent(1));
        Logger.recordOutput("Module Number 4 " , powerDistribution.getCurrent(18));

        for (CANSparkMax m : m_driveArray) {
            Logger.recordOutput("Position drive module " + m.getDeviceId(), m.getEncoder().getPosition());
            Logger.recordOutput("Velocity drive module " + m.getDeviceId(), m.getEncoder().getVelocity());
            Logger.recordOutput("Volts drive module " + m.getDeviceId(), m.getAppliedOutput() * m.getBusVoltage());
        }
        for (CANSparkMax m : m_steerArray) {
            Logger.recordOutput("Position steer module " + m.getDeviceId(), m.getEncoder().getPosition());
            Logger.recordOutput("Velocity steer module " + m.getDeviceId(), m.getEncoder().getVelocity());
            Logger.recordOutput("Volts steer module " + m.getDeviceId(), m.getAppliedOutput() * m.getBusVoltage());
        }
    }
}
