package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ShooterCommand;

// public boolean getSensor() {
//     return sensor.get();
// }
public class Shooter extends SubsystemBase {
    public enum ShooterStates {
    IDLE,
    SourcePickup,
    GroundPickup,
    Shooting,
    StagedForShooting,
    }

    private final CANSparkMax shooter1 = new CANSparkMax(Constants.Shooter.firstShooterID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax shooter2 = new CANSparkMax(Constants.Shooter.secondShooterID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax testIntake = new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless);
    
    private final SparkLimitSwitch shooterFWD;
    private final SparkLimitSwitch shooterReverse;
    
    double scale = 1;

    private ShooterCommand shooterCommand;

    public Shooter(ShooterCommand shooterCommand) {
        shooterFWD = shooter1.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        shooterReverse = shooter1.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        shooter1.setInverted(true);
        shooter2.follow(shooter1, true);
        shooter1.setIdleMode(CANSparkBase.IdleMode.kBrake);
        shooter2.setIdleMode(CANSparkBase.IdleMode.kBrake);
        ShooterStateMachine(ShooterStates.IDLE);
        shooterCommand = this.shooterCommand;
    }

    public void setShooterSpeed(double speed) {
        shooter1.set(scale*speed);
    }
    public void setIntakeSpeed(double speed) {
        testIntake.set(scale*speed);
    }

    public void ShooterStateMachine(ShooterStates state) {
        switch(state) {
            case IDLE:
                setShooterSpeed(0);
                setIntakeSpeed(0);
                break;
            case SourcePickup:
                shooterFWD.enableLimitSwitch(true);
                setShooterSpeed(-0.5);
                break;
            case GroundPickup:
                shooterFWD.enableLimitSwitch(true);
                setIntakeSpeed(0.5);
                setShooterSpeed(0.5);
                if (shooterFWD.isPressed()) {
                    setIntakeSpeed(0);
                    shooterCommand.setWhenPressed(true);
                }
                break;
            case Shooting:
                shooterFWD.enableLimitSwitch(false);
                // setIntakeSpeed(speed);
                setShooterSpeed(0.5);
                break;
            case StagedForShooting:
                setShooterSpeed(0);
                setIntakeSpeed(0);
                break;

            
            default:
                System.out.println("invalid state");
                break;
        }
    }

    public double getShooterSpeed() {
        return shooter1.getEncoder().getVelocity();
    }


}