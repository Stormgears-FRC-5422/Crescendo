package frc.robot.subsystems.drive;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.Drive;

public class ShooterDiagnostic extends SubsystemBase{

  shooter = new CANSparkMax(22, CANSparkLowLevel.MotorType.kBrushless);
  intake = new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless);

  @override
  public void periodic() {
    shooter.set(1);
    intake.set(1)
  }
}