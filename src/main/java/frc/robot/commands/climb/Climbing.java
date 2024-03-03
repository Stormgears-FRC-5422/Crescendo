package frc.robot.commands.climb;

import frc.robot.commands.StormCommand;
import frc.robot.subsystems.Climber;

public class Climbing extends StormCommand {
    Climber m_climber;

    public Climbing(Climber c){
        m_climber = c;
    }

    @Override
    public void initialize() {
        super.initialize();
        m_climber.setClimberState(Climber.ClimberState.CLIMBING);
    }

    @Override
    public void execute(){
        // The subsystem will climb automatically in the CLIMBING state
    }

    @Override
    public boolean isFinished() {
        if (m_climber.isLockedIn()) {
            this.log("Chain is locked in. isFinished = true");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_climber.setClimberState(Climber.ClimberState.IDLE);
        } else {
            m_climber.setClimberState(Climber.ClimberState.HANGING);
        }

        super.end(interrupted);
    }

}
