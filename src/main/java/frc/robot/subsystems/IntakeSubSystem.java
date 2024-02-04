package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.*;


public class IntakeSubSystem extends SubsystemBase {

    private final CANSparkMax testIntake = new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless);

    public void setIntakeSpeed(double speed) {
        testIntake.set(speed);
    }

    public double getIntakeSpeed() {
        return testIntake.getEncoder().getVelocity();
    }

    public Command autoIntake() {
        return Commands.sequence(new InstantCommand(() -> setIntakeSpeed(0.2)),
            new WaitCommand(1.5),
            new InstantCommand(() -> setIntakeSpeed(0)));
    }
}
