package frc.robot.commands.climb;

import frc.robot.commands.StormCommand;
import frc.robot.subsystems.Climber;

public class SimpleGotoDegrees extends StormCommand {
    Climber climber;
    Climber.Direction direction;
    double angle;

    public SimpleGotoDegrees(Climber c, double degrees, Climber.Direction direction){
        addRequirements(c);

        climber = c;
        this.angle = degrees;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        super.initialize();
        climber.setTargetAngle(angle);
        climber.setClimberState(direction == Climber.Direction.FORWARD ?
                                Climber.ClimberState.MOVE_FORWARD : Climber.ClimberState.MOVE_REVERSE);
    }

    @Override
    public void execute(){
        // The subsystem will climb automatically in the CLIMBING state
    }

    @Override
    public boolean isFinished() {
        boolean result = climber.reachedTargetDegrees();
        if (result) {
            this.log("Climber is at the desired position");
        }

        return result;
    }

    @Override
    public void end(boolean interrupted) {
        climber.setClimberState(Climber.ClimberState.IDLE);
        super.end(interrupted);
    }

}
