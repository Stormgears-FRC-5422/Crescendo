package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ShooterCommand;
import frc.utils.motorcontrol.StormSpark;


public class Shooter extends SubsystemBase {

    private final CANSparkMax testShooter = new CANSparkMax(22, CANSparkLowLevel.MotorType.kBrushless);

    public void setShooterSpeed(double speed) {
        testShooter.set(speed);
    }


    public double getShooterSpeed() {
        return testShooter.getEncoder().getVelocity();
    }




}


