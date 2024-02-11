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
    }

    private final CANSparkMax shooterLeadMotor = new CANSparkMax(Constants.Shooter.leaderID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax shooterFollowerMotor = new CANSparkMax(Constants.Shooter.followerID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.Shooter.intakeID, CANSparkLowLevel.MotorType.kBrushless);
    
    private final SparkLimitSwitch shooterForwardLimitSwitch;
    private final SparkLimitSwitch shooterReverseLimitSwitch;
    
    double scale = 1;

    public Shooter() {
        shooterForwardLimitSwitch = shooterLeadMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        shooterReverseLimitSwitch = shooterLeadMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        shooterLeadMotor.setInverted(true);
        shooterFollowerMotor.follow(shooterLeadMotor, true);
        ShooterStateMachine(ShooterStates.IDLE);
    }

    public void ShooterStateMachine(ShooterStates state) {
        switch(state) {
            case IDLE:
                setShooterSpeed(0);
                setIntakeSpeed(0);
                shooterLeadMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
                shooterFollowerMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
                intakeMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
                break;
            case SOURCE_PICKUP:
                shooterForwardLimitSwitch.enableLimitSwitch(true);
                setShooterSpeed(-Constants.Shooter.intakeUpperMotorSpeed);
                break;
            case GROUND_PICKUP:
                shooterForwardLimitSwitch.enableLimitSwitch(true);
                setIntakeSpeed(Constants.Shooter.intakeLowerMotorSpeed);
                setShooterSpeed(Constants.Shooter.intakeUpperMotorSpeed);
                break;
            case SHOOTING:
                shooterForwardLimitSwitch.enableLimitSwitch(false);
                setShooterSpeed(Constants.Shooter.shootMotorSpeed);
                break;
            case STAGED_FOR_SHOOTING:
                setShooterSpeed(0);
                setIntakeSpeed(0);
                shooterLeadMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
                shooterFollowerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
                intakeMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
                break;
            default:
                System.out.println("invalid state");
                break;
        }
    }

    public void setShooterSpeed(double speed) {
        shooterLeadMotor.set(scale*speed);
    }
    public void setIntakeSpeed(double speed) {
        intakeMotor.set(scale*speed);
    }
    public double getShooterSpeed() {
        return shooterLeadMotor.getEncoder().getVelocity();
    }

    public boolean isUpperSensorTriggered() {
        return shooterForwardLimitSwitch.isPressed();
    }

}