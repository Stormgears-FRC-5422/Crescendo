package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class GroundPickup extends Command {
    private final Shooter s;

    public GroundPickup(Shooter s) {
        this.s = s;
        addRequirements(s);
    }

    @Override
    public void initialize() {
        System.out.println("Ground Pickup command running");
        s.ShooterStateMachine(Shooter.ShooterStates.GROUND_PICKUP);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return s.isUpperSensorTriggered();
    }

    @Override
    public void end(boolean interrupted) {
        s.ShooterStateMachine(Shooter.ShooterStates.STAGED_FOR_SHOOTING);
        System.out.println("Ending Ground PIckup Command");
    }

}



