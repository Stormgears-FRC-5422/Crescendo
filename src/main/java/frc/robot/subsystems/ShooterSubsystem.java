package frc.robot.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.revrobotics.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;


public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooter1 = new CANSparkMax(Shooter.firstShooterID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax shooter2 = new CANSparkMax(Shooter.secondShooterID, CANSparkLowLevel.MotorType.kBrushless);


    boolean foo = true;

    public ShooterSubsystem() {
        shooter1.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        shooter1.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        shooter1.setInverted(foo);
        shooter2.follow(shooter1, true);
        shooter1.setIdleMode(CANSparkBase.IdleMode.kBrake);
        shooter2.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    public void setShooterSpeed(double speed) {
        shooter1.set(speed);
    }


    public double getShooterSpeed() {
        return shooter1.getEncoder().getVelocity();
    }
}
