package frc.robot.commands.climb;

import frc.robot.RobotState;
import frc.robot.commands.StormCommand;
import frc.robot.subsystems.Climber;

public class Climbing extends StormCommand {
    Climber climber;
    RobotState state;
    boolean earlyFinish = false;

    public Climbing(Climber c){
        climber = c;
        this.state = RobotState.getInstance();
        addRequirements(c);
    }

    @Override
    public void initialize() {
        super.initialize();

        if (state.climberHasBeenHomed()) {
            climber.setClimberState(Climber.ClimberState.CLIMBING);
            earlyFinish = false;
        } else {
            earlyFinish = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (earlyFinish) {
            log("Finishing early - must home first!");
            return true;
        }

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
