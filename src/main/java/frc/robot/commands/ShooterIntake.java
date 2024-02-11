package frc.robot.commands;

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
        s.ShooterStateMachine(Shooter.ShooterStates.SOURCE_PICKUP);
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
        s.ShooterStateMachine(Shooter.ShooterStates.IDLE);
    }
}



