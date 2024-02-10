package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.joysticks.StormXboxController;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {
    
    private final Shooter s;

    private int counter;

    private boolean whenpressed = false;

    public boolean getWhenPressed(){
        return this.whenpressed;
    }

    public void setWhenPressed(boolean whenpressed){
        this.whenpressed = whenpressed;
    }

    public ShooterCommand(Shooter s) {
        this.s = s;
        addRequirements(s);
    }

    @Override
    public void initialize() {
        System.out.println("Shooter command running");
        s.ShooterStateMachine(Shooter.ShooterStates.Shooting);
        counter = 0;
    }

    @Override
    public void execute() {
        counter++;
    }

    @Override
    public boolean isFinished() {
        return counter > 150;

    }
    
    @Override
    public void end(boolean interrupted) {
        s.ShooterStateMachine(Shooter.ShooterStates.IDLE);
    }

}



