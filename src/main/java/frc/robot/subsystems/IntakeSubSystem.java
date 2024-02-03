package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubSystem extends SubsystemBase {

    private final CANSparkMax testIntake = new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless);

    public void setIntakeSpeed(double speed) {
        testIntake.set(speed);
    }

    public double getIntakeSpeed() {
        return testIntake.getEncoder().getVelocity();
    }
}
