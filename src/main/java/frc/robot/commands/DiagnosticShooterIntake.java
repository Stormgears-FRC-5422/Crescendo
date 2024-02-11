package frc.robot.commands;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class DiagnosticShooterIntake extends Command {

   private Shooter shooter;
    public DiagnosticShooterIntake(Shooter shooter){
      this.shooter = shooter;

      addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.ShooterStateMachine(Shooter.ShooterStates.DIAGNOSTIC);

    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.ShooterStateMachine(Shooter.ShooterStates.IDLE);
    }
}
