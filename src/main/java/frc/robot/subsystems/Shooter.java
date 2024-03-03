package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.utils.vision.NoteVisualizer;

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
        OUTTAKE
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


    double m_shooterMotorSpeed = 0;
    double m_intakeMotorSpeed = 0;

    SlewRateLimiter shooterSlewRateLimiter = new SlewRateLimiter(3);

    public Shooter() {
        shooterLeadMotor = new CANSparkMax(Constants.Shooter.leaderID, CANSparkLowLevel.MotorType.kBrushless);
        shooterFollowerMotor = new CANSparkMax(Constants.Shooter.followerID, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotor = new CANSparkMax(Constants.Shooter.intakeID, CANSparkLowLevel.MotorType.kBrushless);

        shooterLeadMotor.setInverted(true);
        shooterFollowerMotor.follow(shooterLeadMotor, true);
        intakeMotor.setInverted(true);

        shooterForwardLimitSwitch = shooterLeadMotor.getForwardLimitSwitch(Type.kNormallyClosed);
        shooterReverseLimitSwitch = shooterLeadMotor.getReverseLimitSwitch(Type.kNormallyOpen);

        m_robotState = RobotState.getInstance();
        setShooterState(ShooterState.IDLE);
    }

    @Override
    public void periodic() {
//        shooterLeadMotor.set(shooterSlewRateLimiter.calculate(m_shooterMotorSpeed));
        shooterLeadMotor.set(m_shooterMotorSpeed);
        intakeMotor.set(m_intakeMotorSpeed);
        m_robotState.setUpperSensorTriggered(isUpperSensorTriggered());
    }

    public void setShooterState(ShooterState state) {
        m_robotState.setShooterState(state);

        switch (state) {
            case IDLE -> {
                setShooterSpeed(FORWARD, 0);
                setIntakeSpeed(FORWARD, 0);
                setIdleModeAll(IdleMode.kCoast);
            }
            case SOURCE_PICKUP_1 -> {
                setLimitSwitch(REVERSE, false);
                setShooterSpeed(REVERSE, Constants.Shooter.intakeUpperMotorSpeed);
                setIdleModeAll(IdleMode.kBrake);
            }
            case SOURCE_PICKUP_2 -> {
                setLimitSwitch(REVERSE, true);
                setShooterSpeed(REVERSE, Constants.Shooter.intakeUpperMotorSpeed);
                setIdleModeAll(IdleMode.kBrake);
            }
            case GROUND_PICKUP -> {
                setLimitSwitch(FORWARD, true);
                setIntakeSpeed(FORWARD, Constants.Shooter.intakeLowerMotorSpeed);
                setShooterSpeed(FORWARD, Constants.Shooter.intakeUpperMotorSpeed);
                setIdleModeAll(IdleMode.kBrake);
            }
            case SPEAKER_SHOOTING -> {
                setLimitSwitch(FORWARD, false);
                setShooterSpeed(FORWARD, Constants.Shooter.shootMotorSpeed);
                NoteVisualizer.shoot();
            }
            case AMP_SHOOTING -> {
                setLimitSwitch(FORWARD, false);
                setShooterSpeed(FORWARD, Constants.Shooter.ampShootMotorSpeed);
                NoteVisualizer.shoot();
            }
            case OUTTAKE -> {
                setLimitSwitch(REVERSE, false);
                setIntakeSpeed(REVERSE, Constants.Shooter.outtakeSpeed);
                setShooterSpeed(REVERSE, Constants.Shooter.outtakeSpeed);
                setIdleModeAll(IdleMode.kCoast);
            }
            case STAGED_FOR_SHOOTING -> {
                setShooterSpeed(FORWARD, 0);
                setIntakeSpeed(FORWARD, 0);
                setIdleModeAll(IdleMode.kBrake);
            }
            case DIAGNOSTIC -> {
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
                shooterForwardLimitSwitch.enableLimitSwitch(enabled);
            }
            case REVERSE -> {
                shooterReverseLimitSwitch.enableLimitSwitch(enabled);
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
        return shooterForwardLimitSwitch.isPressed();
    }

}
