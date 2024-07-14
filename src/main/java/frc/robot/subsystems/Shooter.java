package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.ShuffleboardConstants;
import frc.utils.LoggerWrapper;
import frc.utils.vision.NoteVisualizer;

import java.util.logging.Logger;

import static frc.robot.subsystems.Shooter.Direction.FORWARD;
import static frc.robot.subsystems.Shooter.Direction.REVERSE;

public class Shooter extends SubsystemBase {
    public enum ShooterState {
        BAD,
        IDLE,
        SOURCE_PICKUP_1,
        SOURCE_PICKUP_2,
        GROUND_PICKUP,
        SPEAKER_SHOOTING,
        STAGED_FOR_SHOOTING,
        DIAGNOSTIC,
        AMP_SHOOTING,
        OUTTAKE,
        EJECT
    }

    public enum Direction {
        FORWARD,
        REVERSE
    }

    private final RobotState m_robotState;

    private final CANSparkMax shooterLeadMotor;
    private final CANSparkMax shooterFollowerMotor;
    private final CANSparkMax intakeMotor;

    private SparkLimitSwitch shooterForwardLimitSwitch;
    private SparkLimitSwitch shooterReverseLimitSwitch;

    private SparkLimitSwitch intakeForwardLimitSwitch;
    private SparkLimitSwitch intakeReverseLimitSwitch;


    double m_shooterMotorSpeed = 0;
    double m_intakeMotorSpeed = 0;

    SlewRateLimiter shooterSlewRateLimiter = new SlewRateLimiter(5);

    Boolean shooterStaged = false;
    ShooterState shooterState;


    public Shooter() {
        shooterLeadMotor = new CANSparkMax(Constants.Shooter.leaderID, CANSparkLowLevel.MotorType.kBrushless);
        shooterFollowerMotor = new CANSparkMax(Constants.Shooter.followerID, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotor = new CANSparkMax(Constants.Shooter.intakeID, CANSparkLowLevel.MotorType.kBrushless);

        shooterLeadMotor.setOpenLoopRampRate(0);
        shooterFollowerMotor.setOpenLoopRampRate(0);
        shooterLeadMotor.setInverted(true);
        shooterFollowerMotor.follow(shooterLeadMotor, true);
        intakeMotor.setInverted(true);
//
//        shooterForwardLimitSwitch = shooterLeadMotor.getForwardLimitSwitch(Type.kNormallyOpen);
//        shooterReverseLimitSwitch = shooterLeadMotor.getReverseLimitSwitch(Type.kNormallyClosed);

        intakeForwardLimitSwitch = intakeMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        intakeReverseLimitSwitch = intakeMotor.getReverseLimitSwitch(Type.kNormallyClosed);

        m_robotState = RobotState.getInstance();
        setShooterState(ShooterState.IDLE);

        ShuffleboardConstants.getInstance().drivetrainTab.addBoolean("Note Stages", () -> shooterStaged);
    }

    @Override
    public void periodic() {
//        shooterLeadMotor.set(shooterSlewRateLimiter.calculate(m_shooterMotorSpeed));
        shooterLeadMotor.set(m_shooterMotorSpeed);
        intakeMotor.set(m_intakeMotorSpeed);
        LoggerWrapper.recordOutput("Lead Shooter RPM", shooterLeadMotor.getEncoder().getVelocity());
        LoggerWrapper.recordOutput("Follower Shooter RPM", shooterFollowerMotor.getEncoder().getVelocity());
        LoggerWrapper.recordOutput("Intake RPM", intakeMotor.getEncoder().getVelocity());
        m_robotState.setUpperSensorTriggered(isUpperSensorTriggered());
    }

    public void setShooterState(ShooterState state) {
        this.shooterState = state;
        m_robotState.setShooterState(state);
//        System.out.println(state);

        switch (state) {
            case IDLE -> {
                shooterStaged = false;
                setShooterSpeed(FORWARD, 0);
                setIntakeSpeed(FORWARD, 0);
                setIdleModeAll(IdleMode.kBrake);
            }
            case SOURCE_PICKUP_1 -> {
                shooterStaged = false;
                setLimitSwitch(REVERSE, false);
                setShooterSpeed(REVERSE, Constants.Shooter.intakeUpperMotorSpeed);
                setIdleModeAll(IdleMode.kBrake);
            }
            case SOURCE_PICKUP_2 -> {
                shooterStaged = false;
                setLimitSwitch(REVERSE, true);
                setShooterSpeed(REVERSE, Constants.Shooter.sourceIntakeMotorSpeed);
                setIdleModeAll(IdleMode.kBrake);
            }
            case GROUND_PICKUP -> {
                shooterStaged = false;
                shooterLeadMotor.setIdleMode(IdleMode.kBrake);
                shooterFollowerMotor.setIdleMode(IdleMode.kBrake);
                setLimitSwitch(FORWARD, true);
                setIntakeSpeed(FORWARD, Constants.Shooter.intakeLowerMotorSpeed);
                setShooterSpeed(FORWARD, Constants.Shooter.intakeUpperMotorSpeed);
                setIdleModeAll(IdleMode.kBrake);
            }
            case SPEAKER_SHOOTING -> {
                shooterStaged = false;
                setLimitSwitch(FORWARD, false);
                if (Constants.Toggles.outReach) {
                    setIntakeSpeed(FORWARD, Constants.Shooter.outReachMotorSpeed);
                    setShooterSpeed(FORWARD, Constants.Shooter.outReachMotorSpeed);
                } else {
                    setIntakeSpeed(FORWARD, Constants.Shooter.intakeMotorSpeed);
                    setShooterSpeed(FORWARD, Constants.Shooter.shootMotorSpeed);
                }
                NoteVisualizer.shoot();
            }
            case AMP_SHOOTING -> {
                shooterStaged = false;
                setLimitSwitch(FORWARD, false);
                if (Constants.Toggles.outReach) {
                    setIntakeSpeed(FORWARD, Constants.Shooter.ampOutReachSpeed);
                    setShooterSpeed(FORWARD, Constants.Shooter.ampOutReachSpeed);
                } else {
                    setShooterSpeed(FORWARD, Constants.Shooter.ampShootMotorSpeed);
                    setIntakeSpeed(FORWARD, Constants.Shooter.ampIntakeMotorSpeed);
                }
                NoteVisualizer.shoot();
            }
            case OUTTAKE -> {
                shooterStaged = false;
                setLimitSwitch(REVERSE, false);
                setIntakeSpeed(REVERSE, Constants.Shooter.outtakeSpeed);
                setShooterSpeed(REVERSE, Constants.Shooter.outtakeSpeed);
                setIdleModeAll(IdleMode.kCoast);
            }
            case EJECT -> {
                shooterStaged = false;
                setLimitSwitch(FORWARD, false);
                setIntakeSpeed(FORWARD, Constants.Shooter.outtakeSpeed);
                setShooterSpeed(FORWARD, Constants.Shooter.outtakeSpeed);
                setIdleModeAll(IdleMode.kCoast);
            }
            case STAGED_FOR_SHOOTING -> {
                shooterStaged = true;
                setShooterSpeed(FORWARD, 0);
                setIntakeSpeed(FORWARD, 0);
                setIdleModeAll(IdleMode.kBrake);
            }
            case DIAGNOSTIC -> {
                shooterStaged = false;
                setLimitSwitch(FORWARD, false);
                setShooterSpeed(FORWARD, Constants.Shooter.diagnosticShooterSpeed);
                setIntakeSpeed(FORWARD, Constants.Shooter.diagnosticIntakeSpeed);
            }
            default -> System.out.println("invalid state");
        }


    }

    private void setIdleModeAll(CANSparkBase.IdleMode mode) {
        shooterLeadMotor.setIdleMode(mode);
        shooterFollowerMotor.setIdleMode(mode);
        intakeMotor.setIdleMode(mode);
    }

    private void setLimitSwitch(Direction d, boolean enabled) {
        switch (d) {
            case FORWARD -> {
//                shooterForwardLimitSwitch.enableLimitSwitch(enabled);
                intakeForwardLimitSwitch.enableLimitSwitch(enabled);
            }
            case REVERSE -> {
//                shooterReverseLimitSwitch.enableLimitSwitch(enabled);
                intakeReverseLimitSwitch.enableLimitSwitch(enabled);
            }
        }
    }

    public void setShooterSpeed(Direction d, double speed) {
        m_shooterMotorSpeed = speed * (d == FORWARD ? 1 : -1);
    }

    public void setIntakeSpeed(Direction d, double speed) {
        m_intakeMotorSpeed = speed * (d == FORWARD ? 1 : -1);
    }

    public double getShooterSpeed() {
        return shooterLeadMotor.getEncoder().getVelocity();
    }

    public boolean isUpperSensorTriggered() {
//        return shooterForwardLimitSwitch.isPressed();
        return intakeForwardLimitSwitch.isPressed();
    }

}
