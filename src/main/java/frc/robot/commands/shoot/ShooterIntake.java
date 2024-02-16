package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterIntake extends Command {
    private final Shooter s;
    private int counter;

    public ShooterIntake(Shooter s) {
        this.s = s;
        addRequirements(s);
    }

    @Override
    public void initialize() {
        System.out.println("Shooter command running");
        s.setShooterState(Shooter.ShooterState.SOURCE_PICKUP_1);
        counter = 0;
    }

    @Override
    public void execute() {
        counter++;
    }

    @Override
    public boolean isFinished() {
        return counter > 30;
    }

    @Override
    public void end(boolean interrupted) {
        s.setShooterState(Shooter.ShooterState.IDLE);
    }
}



