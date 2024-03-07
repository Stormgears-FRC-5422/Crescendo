package frc.robot.commands.climb;

import frc.robot.Constants;
import frc.robot.commands.StormCommand;
import frc.robot.subsystems.Climber;

public class AmpPosition extends StormCommand {
    Climber m_climber;

    private static double MOVE_TO_POSTIION = Constants.Climber.gearRatio * ((Constants.Climber.offsetInDegree + Constants.Climber.ampAngleInDegree + Constants.Climber.overshootInDegree)/360.0);

    public AmpPosition(Climber c){
        m_climber = c;

        addRequirements(c);
    }

    @Override
    public void initialize() {
        super.initialize();
        System.out.println("Soft Limit: " + MOVE_TO_POSTIION);
        m_climber.setReverseSoftLimit((float)MOVE_TO_POSTIION);
        m_climber.setClimberState(Climber.ClimberState.MOVE_REVERSE);
    }

    @Override
    public void execute(){
        // The subsystem will climb automatically in the CLIMBING state
    }

    @Override
    public boolean isFinished() {
        if (m_climber.reachedToReversePosition()) {
            this.log("Climber is at AMP shoot position");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.setClimberState(Climber.ClimberState.IDLE);

        super.end(interrupted);
    }

}
