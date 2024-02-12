package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    public enum ShooterStates {
        BAD,
        IDLE,
        SOURCE_PICKUP,
        SOURCE_PICKUP_1,
        SOURCE_PICKUP_2,
        GROUND_PICKUP,
        SPEAKER_SHOOTING,
        STAGED_FOR_SHOOTING,
        DIAGNOSTIC,
        AMP_SHOOTING,
        OUTTAKE
    }

    private final CANSparkMax shooterLeadMotor;
    private final CANSparkMax shooterFollowerMotor;
    private final CANSparkMax intakeMotor;

    private SparkLimitSwitch shooterForwardLimitSwitch;
    private SparkLimitSwitch shooterReverseLimitSwitch;

    double scale = 1;

    public Shooter() {
        shooterLeadMotor = new CANSparkMax(Constants.Shooter.leaderID, CANSparkLowLevel.MotorType.kBrushless);
        shooterFollowerMotor = new CANSparkMax(Constants.Shooter.followerID, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotor = new CANSparkMax(Constants.Shooter.intakeID, CANSparkLowLevel.MotorType.kBrushless);

        shooterLeadMotor.setInverted(true);
        shooterFollowerMotor.follow(shooterLeadMotor, true);
        intakeMotor.setInverted(true);

        ShooterStateMachine(ShooterStates.IDLE);
    }

    public void ShooterStateMachine(ShooterStates state) {
        switch (state) {
            case IDLE -> {
                setShooterSpeed(0);
                setIntakeSpeed(0);
                setIdleModeAll(IdleMode.kCoast);
            }
            case SOURCE_PICKUP -> {
                setLimitSwitchTypeAll(Type.kNormallyClosed);
                setEnableLimitSwitchAll(false);
                setShooterSpeed(-Constants.Shooter.intakeUpperMotorSpeed);
                setIdleModeAll(IdleMode.kBrake);
            }
            case SOURCE_PICKUP_1 -> {
                setEnableLimitSwitchAll(false);
                setShooterSpeed(-Constants.Shooter.intakeUpperMotorSpeed);
                setIdleModeAll(IdleMode.kBrake);
            }
            case SOURCE_PICKUP_2 -> {
                shooterReverseLimitSwitch = shooterLeadMotor.getReverseLimitSwitch(Type.kNormallyOpen);
                shooterReverseLimitSwitch.enableLimitSwitch(true);
                setShooterSpeed(-Constants.Shooter.intakeUpperMotorSpeed);
                setIdleModeAll(IdleMode.kBrake);
            }
            case GROUND_PICKUP -> {
                setLimitSwitchTypeAll(Type.kNormallyClosed);
                shooterForwardLimitSwitch.enableLimitSwitch(true);
                shooterReverseLimitSwitch.enableLimitSwitch(false);
                setIntakeSpeed(Constants.Shooter.intakeLowerMotorSpeed);
                setShooterSpeed(Constants.Shooter.intakeUpperMotorSpeed);
                setIdleModeAll(IdleMode.kBrake);
            }
            case SPEAKER_SHOOTING -> {
                setLimitSwitchTypeAll(Type.kNormallyClosed);
                setEnableLimitSwitchAll(false);
                setShooterSpeed(Constants.Shooter.shootMotorSpeed);
            }
            case AMP_SHOOTING -> {
                setEnableLimitSwitchAll(false);
                setShooterSpeed(Constants.Shooter.ampShootMotorSpeed);
            }
            case OUTTAKE -> {
                setEnableLimitSwitchAll(false);
                setIntakeSpeed(Constants.Shooter.outtakeSpeed);
                setShooterSpeed(Constants.Shooter.outtakeSpeed);
                setIdleModeAll(IdleMode.kCoast);
            }
            case STAGED_FOR_SHOOTING -> {
                setShooterSpeed(0);
                setIntakeSpeed(0);
                setIdleModeAll(IdleMode.kBrake);
            }
            case DIAGNOSTIC -> {
                shooterForwardLimitSwitch.enableLimitSwitch(false);
                setShooterSpeed(Constants.Shooter.diagnosticShooterSpeed);
                setIntakeSpeed(Constants.Shooter.diagnosticIntakeSpeed);
            }
            default -> System.out.println("invalid state");
        }
    }

    private void setIdleModeAll(CANSparkBase.IdleMode mode) {
        shooterLeadMotor.setIdleMode(mode);
        shooterFollowerMotor.setIdleMode(mode);
        intakeMotor.setIdleMode(mode);
    }

    private void setLimitSwitchTypeAll(SparkLimitSwitch.Type type) {
        shooterForwardLimitSwitch = shooterLeadMotor.getForwardLimitSwitch(type);
        shooterReverseLimitSwitch = shooterLeadMotor.getReverseLimitSwitch(type);
    }

    private void setEnableLimitSwitchAll(boolean set) {
        shooterForwardLimitSwitch.enableLimitSwitch(set);
        shooterReverseLimitSwitch.enableLimitSwitch(set);
    }

    public void setShooterSpeed(double speed) {
        shooterLeadMotor.set(scale * speed);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(scale * speed);
    }

    public double getShooterSpeed() {
        return shooterLeadMotor.getEncoder().getVelocity();
    }

    public boolean isUpperSensorTriggered() {
        return shooterForwardLimitSwitch.isPressed();
    }

}
