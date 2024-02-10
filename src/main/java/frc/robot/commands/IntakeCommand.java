package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.joysticks.StormXboxController;
import frc.robot.subsystems.Shooter;

public class IntakeCommand extends Command {
    
    private boolean whenpressed = false;

    private final Shooter i;

    public boolean getWhenPressed(){
        return this.whenpressed;
    }

    public void setWhenPressed(boolean whenpressed){
        this.whenpressed = whenpressed;
    }

    public IntakeCommand(Shooter i) {
        this.i = i;
        addRequirements(i);
    }

    @Override
    public void initialize() {
        System.out.println("Shooter command running");
        i.ShooterStateMachine(Shooter.ShooterStates.GroundPickup);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return getWhenPressed();

    }
    
    @Override
    public void end(boolean interrupted) {
        i.ShooterStateMachine(Shooter.ShooterStates.IDLE);
    }

}



