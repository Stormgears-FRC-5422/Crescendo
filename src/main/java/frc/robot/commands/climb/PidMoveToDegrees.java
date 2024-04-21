package frc.robot.commands.climb;

import frc.robot.RobotState;
import frc.robot.commands.StormCommand;
import frc.robot.subsystems.Climber;

public class PidMoveToDegrees extends StormCommand {
    Climber climber;
    RobotState state;
    double angle;
    boolean earlyFinish = false;

    public PidMoveToDegrees(Climber c, double degrees){
        climber = c;
        this.angle = degrees;
        this.state = RobotState.getInstance();
        addRequirements(c);
    }

    @Override
    public void initialize() {
        super.initialize();

        if (state.climberHasBeenHomed()) {
            climber.setTargetDegrees(angle);
            climber.setClimberState(Climber.ClimberState.MOVE_PID_POSITION);
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

        boolean result = climber.reachedTargetDegrees();
        if (result) {
            this.log("Climber is at the desired position");
        }
        return result;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted || earlyFinish) {
            climber.setClimberState(Climber.ClimberState.IDLE_BRAKE);
        } else {
            climber.setClimberState(Climber.ClimberState.IDLE_BRAKE);
            // really out to have a background command for holding, not just a state
            // climber.setClimberState(Climber.ClimberState.HOLD_PID_POSITION);
        }
        super.end(interrupted);
    }

}
