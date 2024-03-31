package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Outtake extends Command {
    private final Shooter s;
    private int counter;
    private boolean eject;

    public Outtake(Shooter s, boolean eject) {
        this.s = s;
        this.eject = eject;
        addRequirements(s);
    }

    @Override
    public void initialize() {
        System.out.println("Shooter command running");
        if (eject) {
            s.setShooterState(Shooter.ShooterState.EJECT);
        } else {
            s.setShooterState(Shooter.ShooterState.OUTTAKE);
        }
    }

    @Override
    public void execute() {
        counter++;
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        s.setShooterState(Shooter.ShooterState.IDLE);
    }
}



