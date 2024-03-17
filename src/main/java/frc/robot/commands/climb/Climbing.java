package frc.robot.commands.climb;

import frc.robot.commands.StormCommand;
import frc.robot.subsystems.Climber;

public class Climbing extends StormCommand {
    Climber climber;

    public Climbing(Climber c){
        climber = c;
        addRequirements(c);
    }

    @Override
    public void initialize() {
        super.initialize();
        climber.setClimberState(Climber.ClimberState.CLIMBING);
    }

    @Override
    public boolean isFinished() {
        if (climber.isLockedIn()) {
            this.log("Chain is locked in. isFinished = true");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            climber.setClimberState(Climber.ClimberState.IDLE_BRAKE);
        } else {
            climber.setClimberState(Climber.ClimberState.HANGING);
        }
        super.end(interrupted);
    }

}
