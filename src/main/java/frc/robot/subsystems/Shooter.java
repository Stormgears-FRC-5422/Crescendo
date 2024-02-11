package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// public boolean getSensor() {
//     return sensor.get();
// }
public class Shooter extends SubsystemBase {
    public enum ShooterStates {
        IDLE,
        SOURCE_PICKUP,
        GROUND_PICKUP,
        SHOOTING,
        STAGED_FOR_SHOOTING,
        DIAGNOSTIC,
        AMPSHOOTING,
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
//        shooterFollowerMotor.setInverted(false);
        shooterFollowerMotor.follow(shooterLeadMotor, true);

        intakeMotor.setInverted(true);
        ShooterStateMachine(ShooterStates.IDLE);
    }

    public void ShooterStateMachine(ShooterStates state) {
        switch (state) {
            case IDLE -> {
                setShooterSpeed(0);
                setIntakeSpeed(0);
                shooterLeadMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
                shooterFollowerMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
                intakeMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
            }
            case SOURCE_PICKUP -> {
                shooterForwardLimitSwitch = shooterLeadMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
                shooterReverseLimitSwitch = shooterLeadMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
                shooterReverseLimitSwitch.enableLimitSwitch(false);
                shooterForwardLimitSwitch.enableLimitSwitch(false);
                setShooterSpeed(-Constants.Shooter.intakeUpperMotorSpeed);
            }
            case GROUND_PICKUP -> {
                shooterForwardLimitSwitch = shooterLeadMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
                shooterReverseLimitSwitch = shooterLeadMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

                shooterForwardLimitSwitch.enableLimitSwitch(true);
                shooterReverseLimitSwitch.enableLimitSwitch(false);

//                shooterForwardLimitSwitch.enableLimitSwitch(true);
                setIntakeSpeed(Constants.Shooter.intakeLowerMotorSpeed);
                setShooterSpeed(Constants.Shooter.intakeUpperMotorSpeed);

                shooterLeadMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
                shooterFollowerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
                intakeMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
            }
            case SHOOTING -> {
                shooterForwardLimitSwitch = shooterLeadMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
                shooterReverseLimitSwitch = shooterLeadMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

                shooterForwardLimitSwitch.enableLimitSwitch(false);
                setShooterSpeed(Constants.Shooter.shootMotorSpeed);
            }
            case AMPSHOOTING -> {
                shooterForwardLimitSwitch = shooterLeadMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
                shooterReverseLimitSwitch = shooterLeadMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

                shooterForwardLimitSwitch.enableLimitSwitch(false);
                setShooterSpeed(Constants.Shooter.ampShootMotorSpeed);
            }
            case OUTTAKE -> {
                shooterForwardLimitSwitch = shooterLeadMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
                shooterReverseLimitSwitch = shooterLeadMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

                shooterForwardLimitSwitch.enableLimitSwitch(true);
                shooterReverseLimitSwitch.enableLimitSwitch(false);

//                shooterForwardLimitSwitch.enableLimitSwitch(true);
                setIntakeSpeed(Constants.Shooter.outtakeSpeed);
                setShooterSpeed(Constants.Shooter.outtakeSpeed);

                shooterLeadMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
                shooterFollowerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
                intakeMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

            }
            case STAGED_FOR_SHOOTING -> {
                setShooterSpeed(0);
                setIntakeSpeed(0);
                shooterLeadMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
                shooterFollowerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
                intakeMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
            }
            case DIAGNOSTIC -> {
                shooterForwardLimitSwitch = shooterLeadMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
                shooterReverseLimitSwitch = shooterLeadMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

                shooterForwardLimitSwitch.enableLimitSwitch(false);

                setShooterSpeed(Constants.Shooter.diagnosticShooterSpeed);
                setIntakeSpeed(Constants.Shooter.diagnosticIntakeSpeed);
            }
            default -> System.out.println("invalid state");
        }
    }

    public void setShooterSpeed(double speed) {
        shooterLeadMotor.set(scale*speed);
//        shooterFollowerMotor.set(scale*speed);
    }
    public void setIntakeSpeed(double speed) {
        intakeMotor.set(scale*speed);
    }
    public double getShooterSpeed() {
        return shooterLeadMotor.getEncoder().getVelocity();
    }

    public boolean isUpperSensorTriggered() {
        return shooterForwardLimitSwitch.isPressed();
//        return false;
    }

}
