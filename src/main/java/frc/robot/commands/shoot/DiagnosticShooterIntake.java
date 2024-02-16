package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class DiagnosticShooterIntake extends Command {

   private Shooter shooter;

    public DiagnosticShooterIntake(Shooter shooter){
      this.shooter = shooter;

      addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterState(Shooter.ShooterState.DIAGNOSTIC);

    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterState(Shooter.ShooterState.IDLE);
    }
}
