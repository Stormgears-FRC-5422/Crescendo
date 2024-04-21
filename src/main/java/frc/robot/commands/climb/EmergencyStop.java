package frc.robot.commands.climb;

import frc.robot.commands.StormCommand;
import frc.robot.subsystems.Climber;

public class EmergencyStop extends StormCommand {
    Climber climber;

    public EmergencyStop(Climber c){
        climber = c;
        addRequirements(c);
    }

    @Override
    public void initialize() {
    }

    public void execute(){
        climber.stop();
    }

    public boolean isFinished(){ return true; }

    public void end(boolean interrupted) {
        climber.setClimberState(Climber.ClimberState.IDLE_COAST);
        super.end(interrupted);
    }

}
